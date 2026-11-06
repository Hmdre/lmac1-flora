//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 
// Modificado por André do Valle Medeiros em 2026: intervalo mínimo entre envios calculado pelo tempo no ar e pelo
// duty cycle; fim da espera mínima de 5 s antes do primeiro envio.

#include "SimpleLoRaApp.h"
#include "inet/mobility/static/StationaryMobility.h"
#include "../LoRa/LoRaTagInfo_m.h"
#include "inet/common/packet/Packet.h"
#include <cmath>


namespace flora {

namespace {

// usa o intervalo mínimo, com até 1% de variação aleatória, se a amostra for menor
simtime_t applyMinDelayWithJitter(const cComponent *owner, simtime_t baseDelay, simtime_t minDelay)
{
    simtime_t jitter = minDelay * 0.01;
    if (jitter < SimTime(1, SIMTIME_US))
        jitter = SimTime(1, SIMTIME_US);
    simtime_t sample = minDelay + owner->uniform(SimTime(0, SIMTIME_S), jitter);
    return (sample > baseDelay) ? sample : baseDelay;
}

// Mesmo cálculo de tempo no ar do LoRaTransmitter, inclusive a divisão inteira antes
// do ceil(), para o intervalo mínimo coincidir com a duração real da transmissão.
simtime_t computeTimeOnAir(int spreadFactor, double bandwidthHz, int payloadBytes, int codeRedundance)
{
    double Tsym = std::pow(2, spreadFactor) / (bandwidthHz / 1000);
    int nPreamble = 10;
    simtime_t Tpreamble = (nPreamble + 4.25) * Tsym / 1000;

    int num = 8 * payloadBytes - 4 * spreadFactor + 28 + 16 - 20 * 0;
    int den = 4 * (spreadFactor - 2 * 0);
    int payloadSymbNb = 8 + std::ceil(num / den) * (codeRedundance + 4);
    if (payloadSymbNb < 8)
        payloadSymbNb = 8;
    simtime_t Theader = 0.5 * (8 + payloadSymbNb) * Tsym / 1000;
    simtime_t Tpayload = 0.5 * (8 + payloadSymbNb) * Tsym / 1000;

    return Tpreamble + Theader + Tpayload;
}

} // namespace

Define_Module(SimpleLoRaApp);

void SimpleLoRaApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);
        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
           coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
           StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
           mobility->par("initialX").setDoubleValue(coordsValues.first);
           mobility->par("initialY").setDoubleValue(coordsValues.second);
        }
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;
        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        timeToFirstPacket = par("timeToFirstPacket");
        EV << "Sampled timeToFirstPacket: " << timeToFirstPacket << endl;
        sendMeasurements = new cMessage("sendMeasurements");
        scheduleAt(simTime()+timeToFirstPacket, sendMeasurements);

        sentPackets = 0;
        receivedADRCommands = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaRadio = check_and_cast<LoRaRadio *>(getParentModule()->getSubmodule("LoRaNic")->getSubmodule("radio"));
        loRaRadio->loRaTP = par("initialLoRaTP").doubleValue();
//        setTP(par("initialLoRaTP").doubleValue());
//        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaRadio->loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
//        loRaSF = par("initialLoRaSF");
        loRaRadio->loRaSF = par("initialLoRaSF");
//        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaRadio->loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
//        loRaCR = par("initialLoRaCR");
        loRaRadio->loRaCR = par("initialLoRaCR");
//        loRaUseHeader = par("initialUseHeader");
        loRaRadio->loRaUseHeader = par("initialUseHeader");
        evaluateADRinNode = par("evaluateADRinNode");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");
    }
}

std::pair<double,double> SimpleLoRaApp::generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY)
{
    double randomValueRadius = uniform(0,(radius*radius));
    double randomTheta = uniform(0,2*M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + gatewayX;
    y = gatewayY - y;
    std::pair<double,double> coordValues = std::make_pair(x,y);
    return coordValues;
}

void SimpleLoRaApp::finish()
{
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
    recordScalar("positionX", coord.x);
    recordScalar("positionY", coord.y);
    recordScalar("finalTP", getTP());
    recordScalar("finalSF", getSF());
    recordScalar("sentPackets", sentPackets);
    recordScalar("receivedADRCommands", receivedADRCommands);
}

void SimpleLoRaApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        if (msg == sendMeasurements)
        {
            sendJoinRequest();
            if (simTime() >= getSimulation()->getWarmupPeriod())
                sentPackets++;
            delete msg;
            if(numberOfPacketsToSend == 0 || sentPackets < numberOfPacketsToSend)
            {
                // intervalo mínimo entre envios: tempo no ar dividido pela fração de duty cycle
                simtime_t timeOnAir = computeTimeOnAir(getSF(), getBW().get(), par("dataSize").intValue(), getCR());
                double dutyCyclePercent = par("dutyCyclePercent").doubleValue();
                const simtime_t minNextPacketDelay = timeOnAir / (dutyCyclePercent / 100.0) + SimTime(1, SIMTIME_US);
                timeToNextPacket = applyMinDelayWithJitter(this, par("timeToNextPacket"), minNextPacketDelay);
                sendMeasurements = new cMessage("sendMeasurements");
                scheduleAt(simTime() + timeToNextPacket, sendMeasurements);
            }
        }
    }
    else
    {
        handleMessageFromLowerLayer(msg);
        delete msg;
        //cancelAndDelete(sendMeasurements);
        //sendMeasurements = new cMessage("sendMeasurements");
        //scheduleAt(simTime(), sendMeasurements);
    }
}

void SimpleLoRaApp::handleMessageFromLowerLayer(cMessage *msg)
{
//    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    auto pkt = check_and_cast<Packet *>(msg);
    const auto & packet = pkt->peekAtFront<LoRaAppPacket>();
    if (simTime() >= getSimulation()->getWarmupPeriod())
        receivedADRCommands++;
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT = 0;
        if(packet->getMsgType() == TXCONFIG)
        {
            if(packet->getOptions().getLoRaTP() != -1)
            {
                setTP(packet->getOptions().getLoRaTP());
            }
            if(packet->getOptions().getLoRaSF() != -1)
            {
                setSF(packet->getOptions().getLoRaSF());
            }
            EV << "New TP " << getTP() << endl;
            EV << "New SF " << getSF() << endl;
        }
    }
}

bool SimpleLoRaApp::handleOperationStage(LifecycleOperation *operation, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
    return true;
}

void SimpleLoRaApp::sendJoinRequest()
{
    auto pktRequest = new Packet("DataFrame");
    pktRequest->setKind(DATA);

    auto payload = makeShared<LoRaAppPacket>();
    payload->setChunkLength(B(par("dataSize").intValue()));

    lastSentMeasurement = rand();
    payload->setSampleMeasurement(lastSentMeasurement);

    if(evaluateADRinNode && sendNextPacketWithADRACKReq)
    {
        auto opt = payload->getOptions();
        opt.setADRACKReq(true);
        payload->setOptions(opt);
        //request->getOptions().setADRACKReq(true);
        sendNextPacketWithADRACKReq = false;
    }


    auto loraTag = pktRequest->addTagIfAbsent<LoRaTag>();
    loraTag->setBandwidth(getBW());
    loraTag->setCenterFrequency(getCF());
    loraTag->setSpreadFactor(getSF());
    loraTag->setCodeRendundance(getCR());
    loraTag->setPower(mW(math::dBmW2mW(getTP())));

    //add LoRa control info
  /*  LoRaMacControlInfo *cInfo = new LoRaMacControlInfo();
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    pktRequest->setControlInfo(cInfo);*/

    sfVector.record(getSF());
    tpVector.record(getTP());
    EV << "Wysylam pakiet z TP: " << getTP() << endl;
    EV << "Wysylam pakiet z SF: " << getSF() << endl;
    pktRequest->insertAtBack(payload);
    send(pktRequest, "socketOut");
    if(evaluateADRinNode)
    {
        ADR_ACK_CNT++;
        if(ADR_ACK_CNT == ADR_ACK_LIMIT) sendNextPacketWithADRACKReq = true;
        if(ADR_ACK_CNT >= ADR_ACK_LIMIT + ADR_ACK_DELAY)
        {
            ADR_ACK_CNT = 0;
            increaseSFIfPossible();
        }
    }
    emit(LoRa_AppPacketSent, getSF());
}

void SimpleLoRaApp::increaseSFIfPossible()
{
//    if(loRaSF < 12) loRaSF++;
    if(getSF() < 12) setSF(getSF() + 1);
}

void SimpleLoRaApp::setSF(int SF) {
    loRaRadio->loRaSF = SF;
}

int SimpleLoRaApp::getSF() {
    return loRaRadio->loRaSF;
}

void SimpleLoRaApp::setTP(int TP) {
    loRaRadio->loRaTP = TP;
}

double SimpleLoRaApp::getTP() {
    return loRaRadio->loRaTP;
}

void SimpleLoRaApp::setCF(units::values::Hz CF) {
    loRaRadio->loRaCF = CF;
}

units::values::Hz SimpleLoRaApp::getCF() {
    return loRaRadio->loRaCF;
}

void SimpleLoRaApp::setBW(units::values::Hz BW) {
    loRaRadio->loRaBW = BW;
}

units::values::Hz SimpleLoRaApp::getBW() {
    return loRaRadio->loRaBW;
}

void SimpleLoRaApp::setCR(int CR) {
    loRaRadio->loRaCR = CR;
}

int SimpleLoRaApp::getCR() {
    return loRaRadio->loRaCR;
}

} //end namespace inet
