#!/usr/bin/env python3
"""
Roda as comparações ALOHA x LMAC-1 do omnetpp.ini (raio de 1 km e de 2 km, períodos de
60, 30 e 15 s) e repete cada configuração até a semiamplitude do intervalo de confiança
de 95% da PLR ficar abaixo de 1 ponto percentual, com no mínimo 3 e no máximo 30
repetições. Várias simulações rodam ao mesmo tempo, dentro de um limite de memória e de
núcleos. Se o script for interrompido, ao rodar de novo ele aproveita os .sca que já
estão na pasta de resultados e continua de onde parou.

Requisitos: Linux, Python 3 (só a biblioteca padrão), /usr/bin/time e o FLoRa compilado
(src/run_flora). O script muda para a própria pasta (simulations/) antes de rodar.

Uso:
    python3 rodar_comparacao_final.py
        todas as 12 configurações, resultados em results/comparacao_final
    python3 rodar_comparacao_final.py --configs Comparacao_N100_ALOHA_60s,Comparacao_N100_LMAC_60s
        só as configurações indicadas
    python3 rodar_comparacao_final.py --result-dir results/outra_pasta
        outra pasta de resultados

Saídas: os .sca e um time_<config>_r<n>.log por repetição na pasta de resultados, o
resumo resumo_comparacao_final.csv na mesma pasta e um run_<config>_r<n>.log por
repetição em simulations/.

Máquina usada: Intel Core i5-9400F (6 núcleos), 16 GB de RAM DDR4, com
--ram-budget-gb 11 e --cpu-slots 5, o que deixa memória e um núcleo livres para o
sistema. Cada simulação de 12 h simuladas com 100 nós ocupa cerca de 10 GB com o
LMAC-1 e cerca de 2 GB com o ALOHA, nos dois raios (picos medidos de 8,9 GB e 1,5 GB,
mais uma margem). Esses valores estão em RAM_GB, e o agendador só inicia uma simulação
se a soma das que estão rodando couber no limite.

Em outra máquina, ajuste --ram-budget-gb (memória livre para as simulações, em GB) e
--cpu-slots (simulações simultâneas, em geral um a menos que o número de núcleos).
--ram-budget-gb precisa ser de pelo menos 10; abaixo disso as configurações do LMAC-1
nunca começam e o script fica esperando.
"""

import argparse
import glob
import math
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# --------------------------------------------------------------- parâmetros

SIM_TIME_LIMIT = "43200s"          # 12 h simuladas por repetição
RESULT_DIR = "results/comparacao_final"
REPEAT_CAP = 30                    # teto de repetições por configuração; também vai
                                   # no --repeat=, porque o [General] tem repeat = 10
MIN_REPS = 3                       # com 2 repetições, t = 12,7 deixa o IC largo demais
TARGET_CI_PP = 1.0                 # semiamplitude alvo do IC de 95%, em pontos percentuais
RAM_BUDGET_GB = 11.0               # memória para as simulações, em GB
CPU_SLOTS = 5                      # simulações simultâneas
POLL_INTERVAL = 5                  # segundos entre verificações do agendador
MAX_FAILURES_PER_CONFIG = 3        # falhas aceitas antes de desistir da configuração

# memória estimada por simulação (GB) e repetições simultâneas por configuração
RAM_GB = {"N100_ALOHA": 2.0, "N100_LMAC": 10.0,
          "AreaAmpliada_ALOHA": 2.0, "AreaAmpliada_LMAC": 10.0}
BATCH = {"N100_ALOHA": 3, "N100_LMAC": 1,
         "AreaAmpliada_ALOHA": 3, "AreaAmpliada_LMAC": 1}


def build_configs():
    cfgs = {}
    for period in ("60s", "30s", "15s"):
        for n, proto in (("N100", "ALOHA"), ("N100", "LMAC"),
                         ("AreaAmpliada", "ALOHA"), ("AreaAmpliada", "LMAC")):
            name = f"Comparacao_{n}_{proto}_{period}"
            key = f"{n}_{proto}"
            cfgs[name] = {"ram_gb": RAM_GB[key], "batch": BATCH[key]}
    return cfgs


CONFIGS = build_configs()

# ------------------------------------------------------------------ t-Student

# t(0,975; gl), tabela embutida para não depender do scipy
T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
       8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160,
       14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093,
       20: 2.086, 21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060,
       26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042}


def t_crit(df):
    if df <= 0:
        return float("inf")
    return T95.get(df, 1.960)      # acima de 30 graus de liberdade, usa a normal


def mean(values):
    return sum(values) / len(values)


def ci_half_width(values):
    """Semiamplitude do IC de 95% da média (fração, não pontos percentuais)."""
    n = len(values)
    if n < 2:
        return float("inf")
    m = mean(values)
    var = sum((v - m) ** 2 for v in values) / (n - 1)
    return t_crit(n - 1) * math.sqrt(var) / math.sqrt(n)


def rule_of_three_upper(total_sent):
    """Limite superior unilateral de 95% da PLR quando não houve nenhuma perda.

    Regra de três: sem perdas em N pacotes, p <= 3/N. O IC t-Student não serve nesse
    caso, porque o desvio-padrão é zero. Retorna fração, como ci_half_width().
    """
    if total_sent <= 0:
        return float("inf")
    return 3.0 / total_sent

# ---------------------------------------------------------- leitura do .sca

# Linha: scalar <módulo> <nome> <valor>; o nome vem entre aspas quando tem espaço,
# como em "numReceivedFromNode 42".
SCALAR_RE = re.compile(r'^scalar\s+(\S+)\s+(?:"([^"]*)"|(\S+))\s+(\S+)')
REP_RE = re.compile(r"#(\d+)\.sca$")


def plr_from_sca(path):
    """PLR agregada da repetição: (enviados - recebidos) / enviados.

    Enviados: sentPackets de cada nó. Recebidos: numReceivedFromNode <i> do servidor
    de rede, já sem duplicatas; um nó sem nenhum pacote recebido não tem essa entrada
    e conta como zero.
    """
    sent = 0.0
    recv = 0.0
    n_sent = 0
    n_recv = 0
    with open(path, errors="ignore") as fh:
        for line in fh:
            if not line.startswith("scalar"):
                continue
            m = SCALAR_RE.match(line)
            if not m:
                continue
            module, quoted, plain, value = m.groups()
            name = quoted if quoted is not None else plain
            if name == "sentPackets" and ".loRaNodes[" in module:
                sent += float(value)
                n_sent += 1
            elif (name.startswith("numReceivedFromNode")
                  and ".networkServer.app[" in module):
                recv += float(value)
                n_recv += 1

    if n_sent == 0 or sent <= 0:
        raise ValueError(f"{os.path.basename(path)}: nenhum sentPackets > 0 "
                         f"(a run chegou ao finish()?)")
    if recv > sent:
        raise ValueError(f"{os.path.basename(path)}: recebidos ({recv:.0f}) > "
                         f"enviados ({sent:.0f}) -- deduplicacao suspeita")
    return (sent - recv) / sent, sent, recv, n_sent, n_recv


def find_sca(result_dir, cfg, rep):
    for path in glob.glob(os.path.join(result_dir, f"{cfg}-*.sca")):
        m = REP_RE.search(path)
        if m and int(m.group(1)) == rep:
            return path
    return None


def scan_existing(result_dir, cfg):
    out = {}
    for path in glob.glob(os.path.join(result_dir, f"{cfg}-*.sca")):
        m = REP_RE.search(path)
        if m:
            out[int(m.group(1))] = path
    return out

# ------------------------------------------------------------------- estado

def log(msg):
    print(f"[{datetime.now():%H:%M:%S}] {msg}", flush=True)


class ConfigState:
    def __init__(self, name, spec):
        self.name = name
        self.ram_gb = spec["ram_gb"]
        self.batch = spec["batch"]
        self.plrs = []            # PLRs das repetições concluídas
        self.total_sent = 0.0     # pacotes enviados, somados nas repetições
        self.total_lost = 0.0     # pacotes perdidos (enviados - recebidos), somados
        self.done = set()         # repetições com .sca válido
        self.failed = set()       # repetições que falharam
        self.inflight = {}        # repetição -> processo, arquivo de log e início
        self.n_failures = 0
        self.converged = False
        self.gave_up = False

    def record(self, plr, sent, recv):
        self.plrs.append(plr)
        self.total_sent += sent
        self.total_lost += sent - recv

    # --- estatística ---
    @property
    def n(self):
        return len(self.plrs)

    @property
    def plr_mean(self):
        return mean(self.plrs) if self.plrs else float("nan")

    @property
    def zero_loss(self):
        """Nenhuma perda em todas as repetições concluídas (contagem inteira de pacotes)."""
        return self.n > 0 and self.total_lost == 0

    @property
    def ci_pp(self):
        """Semiamplitude do IC bilateral, em pontos percentuais (só vale com perdas)."""
        return ci_half_width(self.plrs) * 100.0

    @property
    def upper_pp(self):
        """Limite superior unilateral de 95% (regra de três), em %."""
        return rule_of_three_upper(self.total_sent) * 100.0

    @property
    def precision_pp(self):
        """Valor comparado com a meta: semiamplitude do IC ou, sem perdas, o limite superior."""
        return self.upper_pp if self.zero_loss else self.ci_pp

    def interval_text(self):
        """Texto do intervalo para o log e o resumo."""
        if self.n == 0:
            return "--"
        if self.zero_loss:
            return f"<={self.upper_pp:.3f}% (unilateral)"
        ci = self.ci_pp
        return "n/d" if math.isinf(ci) else f"+-{ci:.3f}pp"

    def reevaluate(self, target_ci_pp, min_reps):
        """Marca a convergência: ao menos min_reps repetições e precisão abaixo da meta.

        Sem perdas, usa o limite da regra de três, para não repetir indefinidamente à
        espera de uma perda.
        """
        if self.n >= min_reps and self.precision_pp < target_ci_pp:
            self.converged = True

    # --- agendamento ---
    def launched_count(self):
        return len(self.done) + len(self.inflight)

    def can_launch(self, cap):
        if self.converged or self.gave_up:
            return False
        if len(self.inflight) >= self.batch:
            return False
        return self.launched_count() < cap and self.next_rep(cap) is not None

    def next_rep(self, cap):
        used = self.done | set(self.inflight) | self.failed
        for i in range(cap):
            if i not in used:
                return i
        return None

    def status(self, cap):
        if self.gave_up:
            return "FALHOU"
        if self.converged:
            return "convergiu"
        if self.launched_count() >= cap or self.next_rep(cap) is None:
            return "NAO convergiu (teto)"
        return "pendente"

# ---------------------------------------------------------------- agendador

class Campaign:
    def __init__(self, args):
        self.args = args
        self.states = {name: ConfigState(name, CONFIGS[name])
                       for name in args.configs}
        self.stopping = False

    # --- retomada ---
    def resume(self):
        for st in self.states.values():
            for rep, path in sorted(scan_existing(self.args.result_dir,
                                                  st.name).items()):
                try:
                    plr, sent, recv, _, _ = plr_from_sca(path)
                except ValueError as exc:
                    log(f"AVISO  {st.name} r{rep}: .sca existente ignorado "
                        f"({exc}); a repeticao sera refeita")
                    continue
                st.record(plr, sent, recv)
                st.done.add(rep)
            if st.done:
                st.reevaluate(self.args.target_ci_pp, self.args.min_reps)
                log(f"RETOMA {st.name}: {st.n} repeticao(oes) reaproveitada(s) "
                    f"-- PLR {st.plr_mean*100:.3f}% "
                    f"{st.interval_text()}"
                    f"{' [convergiu]' if st.converged else ''}")

    # --- contabilidade ---
    def ram_in_flight(self):
        return sum(len(st.inflight) * st.ram_gb for st in self.states.values())

    def jobs_in_flight(self):
        return sum(len(st.inflight) for st in self.states.values())

    # --- execução ---
    def launch(self, st, rep):
        run_log = f"run_{st.name}_r{rep}.log"
        time_log = os.path.join(self.args.result_dir,
                                f"time_{st.name}_r{rep}.log")
        cmd = ["/usr/bin/time", "-v", "-o", time_log,
               "../src/run_flora",
               "-f", "omnetpp.ini",
               "-c", st.name,
               "-r", str(rep),
               "-u", "Cmdenv",
               f"--sim-time-limit={self.args.sim_time_limit}",
               f"--result-dir={self.args.result_dir}",
               f"--repeat={self.args.cap}"]
        fh = open(run_log, "w")
        proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT)
        st.inflight[rep] = {"proc": proc, "fh": fh, "started": time.time()}
        log(f"INICIA {st.name} r{rep} "
            f"({st.ram_gb:.1f}GB; em voo: {self.jobs_in_flight()}/"
            f"{self.args.cpu_slots} jobs, "
            f"{self.ram_in_flight():.1f}/{self.args.ram_budget_gb:.1f}GB)")

    def reap(self):
        for st in self.states.values():
            for rep, job in list(st.inflight.items()):
                rc = job["proc"].poll()
                if rc is None:
                    continue
                job["fh"].close()
                del st.inflight[rep]
                elapsed = time.time() - job["started"]
                sca = find_sca(self.args.result_dir, st.name, rep)
                if rc != 0 or sca is None:
                    self.mark_failure(st, rep,
                                      f"exit {rc}, .sca "
                                      f"{'ausente' if sca is None else 'ok'}")
                    continue
                try:
                    plr, sent, recv, n_sent, n_recv = plr_from_sca(sca)
                except ValueError as exc:
                    self.mark_failure(st, rep, str(exc))
                    continue

                st.record(plr, sent, recv)
                st.done.add(rep)
                st.reevaluate(self.args.target_ci_pp, self.args.min_reps)
                log(f"OK     {st.name} r{rep} ({elapsed/60:.1f}min) -- "
                    f"PLR {plr*100:.3f}% "
                    f"(env {sent:.0f}/{n_sent} nos, rec {recv:.0f}/{n_recv}) "
                    f"| n={st.n} media {st.plr_mean*100:.3f}% "
                    f"{st.interval_text()}"
                    f"{' -> CONVERGIU' if st.converged else ''}")

    def mark_failure(self, st, rep, detail):
        st.failed.add(rep)
        st.n_failures += 1
        log(f"FALHA  {st.name} r{rep}: {detail} "
            f"({st.n_failures}/{self.args.max_failures})")
        if st.n_failures >= self.args.max_failures:
            st.gave_up = True
            log(f"DESIST {st.name}: {st.n_failures} falhas, Config abandonada")

    def fill(self):
        """Inicia simulações até esgotar o limite de memória ou de núcleos.

        As mais pesadas entram primeiro; senão as leves ocupariam a memória que o
        LMAC-1 precisa.
        """
        while not self.stopping:
            if self.jobs_in_flight() >= self.args.cpu_slots:
                return
            candidates = [st for st in self.states.values()
                          if st.can_launch(self.args.cap)
                          and (self.ram_in_flight() + st.ram_gb
                               <= self.args.ram_budget_gb)]
            if not candidates:
                return
            candidates.sort(key=lambda s: -s.ram_gb)
            st = candidates[0]
            self.launch(st, st.next_rep(self.args.cap))

    def pending(self):
        return any(st.can_launch(self.args.cap) for st in self.states.values())

    def run(self):
        self.resume()
        if not self.pending() and self.jobs_in_flight() == 0:
            log("Nada a fazer -- todas as Configs ja convergiram ou bateram "
                "o teto.")
        while not self.stopping and (self.pending()
                                     or self.jobs_in_flight() > 0):
            self.reap()
            self.fill()
            if self.jobs_in_flight() > 0 or self.pending():
                time.sleep(self.args.poll)
        self.reap()

    def kill_all(self):
        self.stopping = True
        for st in self.states.values():
            for rep, job in st.inflight.items():
                log(f"Encerrando {st.name} r{rep}...")
                job["proc"].terminate()
        for st in self.states.values():
            for job in st.inflight.values():
                try:
                    job["proc"].wait(timeout=15)
                except subprocess.TimeoutExpired:
                    job["proc"].kill()
                job["fh"].close()

    # --- relatório ---
    def report(self):
        print()
        print("=" * 92)
        print(f"RESUMO DA CAMPANHA  (meta: incerteza 95% < "
              f"{self.args.target_ci_pp}pp"
              f" | teto: {self.args.cap} repeticoes | "
              f"sim-time-limit={self.args.sim_time_limit})")
        print("=" * 92)
        print(f"{'Config':<32}{'reps':>5}{'PLR media':>13}{'incerteza 95%':>22}"
              f"   {'situacao'}")
        print("-" * 92)
        zero_loss_rows = []
        for name in self.args.configs:
            st = self.states[name]
            if st.n == 0:
                print(f"{name:<32}{0:>5}{'--':>13}{'--':>22}   "
                      f"{st.status(self.args.cap)}")
                continue
            if st.zero_loss:
                zero_loss_rows.append(st)
            print(f"{name:<32}{st.n:>5}{st.plr_mean*100:>12.3f}%"
                  f"{st.interval_text():>22}   {st.status(self.args.cap)}")
        print("-" * 92)
        for st in zero_loss_rows:
            print(f"  {st.name}: 0 perdas observadas em {st.n} repeticao(oes) "
                  f"({st.total_sent:.0f} pacotes totais); "
                  f"limite superior 95%: {st.upper_pp:.3f}%")
        if zero_loss_rows:
            print("  (limite unilateral pela regra de tres -- nao e um IC "
                  "bilateral; ver rule_of_three_upper())")
            print("-" * 92)
        self.write_csv()

    def write_csv(self):
        path = os.path.join(self.args.result_dir, "resumo_comparacao_final.csv")
        with open(path, "w") as fh:
            fh.write("config,n_reps,plr_media_pct,criterio,ic95_pp,"
                     "limite_sup_95_pct,total_enviados,situacao,plrs_pct\n")
            for name in self.args.configs:
                st = self.states[name]
                plrs = ";".join(f"{p*100:.4f}" for p in st.plrs)
                media = "" if st.n == 0 else f"{st.plr_mean*100:.4f}"
                if st.n == 0:
                    criterio, ci_txt, ub_txt = "", "", ""
                elif st.zero_loss:
                    criterio = "regra_de_tres_0_perdas"
                    ci_txt = ""
                    ub_txt = f"{st.upper_pp:.4f}"
                else:
                    ci = st.ci_pp
                    criterio = "ic_bilateral_t"
                    ci_txt = "" if math.isinf(ci) else f"{ci:.4f}"
                    ub_txt = ""
                fh.write(f"{name},{st.n},{media},{criterio},{ci_txt},{ub_txt},"
                         f"{st.total_sent:.0f},{st.status(self.args.cap)},"
                         f"{plrs}\n")
        log(f"Resumo gravado em {path}")

# ------------------------------------------------------------------- main

def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Campanha final ALOHA vs. LMAC com parada por IC 95%.")
    p.add_argument("--sim-time-limit", default=SIM_TIME_LIMIT,
                   help=f"default: {SIM_TIME_LIMIT} (12h simuladas)")
    p.add_argument("--result-dir", default=RESULT_DIR,
                   help=f"default: {RESULT_DIR}")
    p.add_argument("--cap", type=int, default=REPEAT_CAP,
                   help=f"teto de repeticoes por Config (default: {REPEAT_CAP})")
    p.add_argument("--min-reps", type=int, default=MIN_REPS,
                   help=f"default: {MIN_REPS}")
    p.add_argument("--target-ci-pp", type=float, default=TARGET_CI_PP,
                   help=f"default: {TARGET_CI_PP}")
    p.add_argument("--ram-budget-gb", type=float, default=RAM_BUDGET_GB,
                   help=f"default: {RAM_BUDGET_GB}")
    p.add_argument("--cpu-slots", type=int, default=CPU_SLOTS,
                   help=f"default: {CPU_SLOTS}")
    p.add_argument("--poll", type=int, default=POLL_INTERVAL,
                   help=f"default: {POLL_INTERVAL}s")
    p.add_argument("--max-failures", type=int, default=MAX_FAILURES_PER_CONFIG,
                   help=f"default: {MAX_FAILURES_PER_CONFIG}")
    p.add_argument("--configs", default="",
                   help="subconjunto separado por virgula (default: as 12)")
    args = p.parse_args(argv)

    if args.configs:
        names = [c.strip() for c in args.configs.split(",") if c.strip()]
        desconhecidas = [c for c in names if c not in CONFIGS]
        if desconhecidas:
            p.error(f"Config(s) desconhecida(s): {', '.join(desconhecidas)}")
        args.configs = names
    else:
        args.configs = list(CONFIGS)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    os.chdir(Path(__file__).resolve().parent)
    os.makedirs(args.result_dir, exist_ok=True)

    campaign = Campaign(args)

    def handler(signum, frame):
        log("Interrompido -- encerrando processos filhos...")
        campaign.kill_all()
        campaign.report()
        sys.exit(1)

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    log(f"{len(args.configs)} Config(s) | orcamento "
        f"{args.ram_budget_gb:.1f}GB / {args.cpu_slots} jobs | "
        f"meta IC +-{args.target_ci_pp}pp | teto {args.cap} repeticoes")
    campaign.run()
    campaign.report()
    return 0


if __name__ == "__main__":
    sys.exit(main())
