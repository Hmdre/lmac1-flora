//
// Copyright (C) 2016 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//
// Derivado de LoRaMac.cc do FLoRa; modificado por André do Valle Medeiros em 2026: implementação do LMAC-1.

#include "inet/common/ModuleAccess.h"
#include "inet/linklayer/common/Ieee802Ctrl.h"
#include "inet/linklayer/common/UserPriority.h"
#include "inet/linklayer/common/MacAddressTag_m.h"
#include "inet/linklayer/csmaca/CsmaCaMac.h"
#include "Lmac1Mac.h"
#include "LoRaTagInfo_m.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/linklayer/csmaca/CsmaCaMacHeader_m.h"


namespace flora {

Define_Module(Lmac1Mac);

Lmac1Mac::~Lmac1Mac()
{
    cancelAndDelete(cadTimer);
    cancelAndDelete(endTransmission);
    cancelAndDelete(endReception);
    cancelAndDelete(droppedPacket);
    cancelAndDelete(endDelay_1);
    cancelAndDelete(endListening_1);
    cancelAndDelete(endDelay_2);
    cancelAndDelete(endListening_2);
    cancelAndDelete(mediumStateChange);
}

/****************************************************************
 * Initialization functions.
 */
void Lmac1Mac::initialize(int stage)
{
    MacProtocolBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        EV << "Initializing stage 0\n";

        headerLength = par("headerLength");
        ackLength = par("ackLength");
        ackTimeout = par("ackTimeout");
        retryLimit = par("retryLimit");

        waitDelay1Time = 1;
        listening1Time = 1;
        waitDelay2Time = 1;
        listening2Time = par("listening2Time");
        cadDuration = par("cadDuration");

        const char *addressString = par("address");
        if (!strcmp(addressString, "auto")) {
            address = MacAddress::generateAutoAddress();
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);

        cModule *radioModule = getModuleFromPar<cModule>(par("radioModule"), this);
        radioModule->subscribe(IRadio::receptionStateChangedSignal, this);
        radioModule->subscribe(IRadio::transmissionStateChangedSignal, this);
        radioModule->subscribe(LoRaRadio::droppedPacket, this);
        radio = check_and_cast<IRadio *>(radioModule);

        cadTimer = new cMessage("CAD_Timer");
        endTransmission = new cMessage("Transmission");
        endReception = new cMessage("Reception");
        droppedPacket = new cMessage("Dropped Packet");
        endDelay_1 = new cMessage("Delay_1");
        endListening_1 = new cMessage("Listening_1");
        endDelay_2 = new cMessage("Delay_2");
        endListening_2 = new cMessage("Listening_2");
        mediumStateChange = new cMessage("MediumStateChange");

        txQueue = getQueue(gate(upperLayerInGateId));

        fsm.setName("Lmac1Mac State Machine");
        backoffPeriod = -1;
        retryCounter = 0;
        sequenceNumber = 0;

        numRetry = 0;
        numSentWithoutRetry = 0;
        numGivenUp = 0;
        numCollision = 0;
        numSent = 0;
        numReceived = 0;
        numSentBroadcast = 0;
        numReceivedBroadcast = 0;

        WATCH(fsm);
        WATCH(backoffPeriod);
        WATCH(retryCounter);
        WATCH(numRetry);
        WATCH(numSentWithoutRetry);
        WATCH(numGivenUp);
        WATCH(numCollision);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numSentBroadcast);
        WATCH(numReceivedBroadcast);
    }
    else if (stage == INITSTAGE_LINK_LAYER)
        turnOnReceiver();
}

void Lmac1Mac::finish()
{
    recordScalar("numRetry", numRetry);
    recordScalar("numSentWithoutRetry", numSentWithoutRetry);
    recordScalar("numGivenUp", numGivenUp);
    recordScalar("numSent", numSent);
    recordScalar("numReceived", numReceived);
    recordScalar("numSentBroadcast", numSentBroadcast);
    recordScalar("numReceivedBroadcast", numReceivedBroadcast);
}

void Lmac1Mac::configureNetworkInterface()
{
    networkInterface->setDatarate(bitrate);
    networkInterface->setMacAddress(address);
    networkInterface->setMtu(std::numeric_limits<int>::quiet_NaN());
    networkInterface->setMulticast(true);
    networkInterface->setBroadcast(true);
    networkInterface->setPointToPoint(false);
}

/****************************************************************
 * Message handling functions.
 */
void Lmac1Mac::handleSelfMessage(cMessage *msg)
{
    EV << "received self message: " << msg << endl;
    handleWithFsm(msg);
}

void Lmac1Mac::handleUpperPacket(Packet *packet)
{
    if(fsm.getState() != IDLE) {
         error("Wrong, it should not happen erroneous state: %s", fsm.getStateName());
    }
    packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::apskPhy);

    EV << "frame " << packet << " received from higher layer " << endl;
    auto pktEncap = encapsulate(packet);

    const auto &frame = pktEncap->peekAtFront<LoRaMacFrame>();
    if (frame == nullptr)
        throw cRuntimeError("Header LoRaMacFrame not found");

    if (currentTxFrame != nullptr)
        throw cRuntimeError("Model error: incomplete transmission exists");
    currentTxFrame = pktEncap;
    handleWithFsm(currentTxFrame);
}


void Lmac1Mac::handleLowerPacket(Packet *msg)
{
    if( (fsm.getState() == RECEIVING_1) || (fsm.getState() == RECEIVING_2)) handleWithFsm(msg);
    else delete msg;
}

void Lmac1Mac::processUpperPacket()
{
    Packet *packet = dequeuePacket();
    handleUpperMessage(packet);
}

queueing::IPassivePacketSource *Lmac1Mac::getProvider(cGate *gate)
{
    return (gate->getId() == upperLayerInGateId) ? txQueue.get() : nullptr;
}

void Lmac1Mac::handleCanPullPacketChanged(cGate *gate)
{
    Enter_Method("handleCanPullPacketChanged");
    if (fsm.getState() == IDLE && !txQueue->isEmpty()) {
        processUpperPacket();
    }
}

void Lmac1Mac::handlePullPacketProcessed(Packet *packet, cGate *gate, bool successful)
{
    Enter_Method("handlePullPacketProcessed");
    throw cRuntimeError("Not supported callback");
}

void Lmac1Mac::handleWithFsm(cMessage *msg)
{
    // As trocas de modo do rádio chamam handleWithFsm() de novo pelo sinal do receptor;
    // só a chamada mais externa retoma o próximo pacote no fim da função.
    bool isReentrant = processingFsm;
    processingFsm = true;

    Ptr<LoRaMacFrame>frame = nullptr;

    auto pkt = dynamic_cast<Packet *>(msg);
    if (pkt) {
        const auto &chunk = pkt->peekAtFront<Chunk>();
        frame = dynamicPtrCast<LoRaMacFrame>(constPtrCast<Chunk>(chunk));
    }
    FSMA_Switch(fsm)
    {
        FSMA_State(IDLE)
        {
            // O rádio fica em recepção também no IDLE: o INET só decide se recebe um
            // sinal no instante em que ele começa a chegar.

            // pacote novo da aplicação ou retirado da fila: começa o DIFS
            FSMA_Event_Transition(Idle-DifsSensing,
                                  isUpperMessage(msg) || msg == currentTxFrame,
                                  DIFS_SENSING,
                
                EV << "LMAC-1: Packet received from upper layer. Starting DIFS sensing.\n";
                isBackoffGenerated = false; // o próximo backoff será sorteado
                difsCount = 0;
                turnOnReceiver();
                
                // primeiro CAD do DIFS
                scheduleAt(simTime() + cadDuration, cadTimer);
            );
        }

        FSMA_State(DIFS_SENSING)
        {
            // CAD ocupado: recomeça o DIFS
            FSMA_Event_Transition(Difs-Ocupado,
                                  msg == cadTimer && isReceiving(),
                                  DIFS_SENSING,

                EV << "LMAC-1: Channel BUSY during DIFS. Resetting DIFS count.\n"; 
                difsCount = 0;
                scheduleAt(simTime() + cadDuration, cadTimer); 
            );

            // CAD livre, DIFS ainda incompleto (12 CADs)
            FSMA_Event_Transition(Difs-Livre-Continua,
                                  msg == cadTimer && !isReceiving() && difsCount < 11,
                                  DIFS_SENSING,
                difsCount++;
                EV << "LMAC-1: DIFS channel FREE. Count: " << difsCount << "/12\n";
                scheduleAt(simTime() + cadDuration, cadTimer);
            );

            // 12º CAD livre: DIFS completo, passa ao backoff
            FSMA_Event_Transition(Difs-Livre-Fim,
                                  msg == cadTimer && !isReceiving() && difsCount >= 11,
                                  BACKOFF_SENSING,
                difsCount++;
                
                // um backoff interrompido é retomado; só se sorteia outro quando não
                // há um pendente
                if (!isBackoffGenerated) {
                    currentBackoff = intuniform(4, 64); // em CADs
                    isBackoffGenerated = true;
                    EV << "LMAC-1: DIFS cleared! Generating new Backoff: " << currentBackoff << "\n";
                } else {
                    EV << "LMAC-1: DIFS cleared! Resuming paused Backoff from: " << currentBackoff << "\n";
                }
                scheduleAt(simTime() + cadDuration, cadTimer);
            );
        }

        FSMA_State(BACKOFF_SENSING)
        {
            // CAD ocupado no backoff: congela a contagem e volta ao DIFS
            FSMA_Event_Transition(Backoff-Ocupado,
                                  msg == cadTimer && isReceiving(),
                                  DIFS_SENSING,
                
                EV << "LMAC-1: Channel BUSY during Backoff! Pausing countdown at " << currentBackoff << ". Returning to DIFS.\n";
                difsCount = 0; // currentBackoff é mantido para ser retomado
                scheduleAt(simTime() + cadDuration, cadTimer);
            );

            // CAD livre: decrementa o backoff
            FSMA_Event_Transition(Backoff-Livre-Continua,
                                  msg == cadTimer && !isReceiving() && currentBackoff > 1,
                                  BACKOFF_SENSING,
                currentBackoff--;
                EV << "LMAC-1: Backoff channel FREE. Countdown: " << currentBackoff << "\n";
                scheduleAt(simTime() + cadDuration, cadTimer);
            );

            // backoff zerado com o canal livre: transmite
            FSMA_Event_Transition(Backoff-Livre-Fim,
                                  msg == cadTimer && !isReceiving() && currentBackoff <= 1,
                                  TRANSMIT,
                currentBackoff--;
                EV << "LMAC-1: Backoff finished! Ready to TRANSMIT.\n";
            );
        }

        FSMA_State(TRANSMIT)
        {
            // A troca de modo do rádio em sendDataFrame() chama handleWithFsm() de novo;
            // a guarda processingFsm e o FSMA_Enter impedem um segundo envio.
            FSMA_Enter(sendDataFrame(getCurrentTransmission()));
            FSMA_Event_Transition(Transmit-Wait_Delay_1,
                                  msg == endTransmission,
                                  WAIT_DELAY_1,

                EV << "LMAC-1: Frame physically transmitted! Resetting MAC variables.\n";
                
                // zera o estado do DIFS e do backoff para o próximo pacote
                isBackoffGenerated = false;
                currentBackoff = 0;
                difsCount = 0;

                finishCurrentTransmission();
                numSent++;
            );
        }
        FSMA_State(WAIT_DELAY_1)
        {
            // o rádio já voltou à recepção no fim da transmissão (ver receiveSignal())
            FSMA_Event_Transition(Wait_Delay_1-Listening_1,
                                  msg == endDelay_1 || endDelay_1->isScheduled() == false,
                                  LISTENING_1,
            );
        }
        FSMA_State(LISTENING_1)
        {
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(Listening_1-Wait_Delay_2,
                                  msg == endListening_1 || endListening_1->isScheduled() == false,
                                  WAIT_DELAY_2,
            );
            FSMA_Event_Transition(Listening_1-Receiving1,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING_1,
            );
        }
        FSMA_State(RECEIVING_1)
        {
            FSMA_Event_Transition(Receive-Unicast-Not-For,
                                  isLowerMessage(msg) && !isForUs(frame),
                                  LISTENING_1,
            );
            FSMA_Event_Transition(Receive-Unicast,
                                  isLowerMessage(msg) && isForUs(frame),
                                  IDLE,
                sendUp(decapsulate(pkt));
                numReceived++;
                cancelEvent(endListening_1);
                cancelEvent(endDelay_2);
                cancelEvent(endListening_2);
            );
            FSMA_Event_Transition(Receive-BelowSensitivity,
                                  msg == droppedPacket,
                                  LISTENING_1,
            );

        }
        FSMA_State(WAIT_DELAY_2)
        {
            // como em WAIT_DELAY_1, o rádio continua em recepção
            FSMA_Event_Transition(Wait_Delay_2-Listening_2,
                                  msg == endDelay_2 || endDelay_2->isScheduled() == false,
                                  LISTENING_2,
            );
        }
        FSMA_State(LISTENING_2)
        {
            FSMA_Enter(turnOnReceiver());
            FSMA_Event_Transition(Listening_2-idle,
                                  msg == endListening_2 || endListening_2->isScheduled() == false,
                                  IDLE,
            );
            FSMA_Event_Transition(Listening_2-Receiving2,
                                  msg == mediumStateChange && isReceiving(),
                                  RECEIVING_2,
            );
        }
        FSMA_State(RECEIVING_2)
        {
            FSMA_Event_Transition(Receive2-Unicast-Not-For,
                                  isLowerMessage(msg) && !isForUs(frame),
                                  LISTENING_2,
            );
            FSMA_Event_Transition(Receive2-Unicast,
                                  isLowerMessage(msg) && isForUs(frame),
                                  IDLE,
                sendUp(pkt);
                numReceived++;
                cancelEvent(endListening_2);
            );
            FSMA_Event_Transition(Receive2-BelowSensitivity,
                                  msg == droppedPacket,
                                  LISTENING_2,
            );
        }
    }

    if (!isReentrant && fsm.getState() == IDLE) {
        if (currentTxFrame != nullptr)
            handleWithFsm(currentTxFrame);
        else if (!txQueue->isEmpty()) {
            processUpperPacket();
        }
    }

    if (endSifs) {
        if (isLowerMessage(msg) && pkt->getOwner() == this && (endSifs->getContextPointer() != pkt))
            delete pkt;
    }
    else {
        if (isLowerMessage(msg) && pkt->getOwner() == this)
            delete pkt;
    }
    getDisplayString().setTagArg("t", 0, fsm.getStateName());

    if (!isReentrant)
        processingFsm = false;
}

void Lmac1Mac::receiveSignal(cComponent *source, simsignal_t signalID, intval_t value, cObject *details)
{
    Enter_Method_Silent();
    if (signalID == IRadio::receptionStateChangedSignal) {
        IRadio::ReceptionState newRadioReceptionState = (IRadio::ReceptionState)value;
        if (receptionState == IRadio::RECEPTION_STATE_RECEIVING) {
            // continua em recepção em vez de dormir, para o CAD não perder sinais
            turnOnReceiver();
        }
        receptionState = newRadioReceptionState;
        handleWithFsm(mediumStateChange);
    }
    else if (signalID == LoRaRadio::droppedPacket) {
        turnOnReceiver();
        handleWithFsm(droppedPacket);
    }
    else if (signalID == IRadio::transmissionStateChangedSignal) {
        IRadio::TransmissionState newRadioTransmissionState = (IRadio::TransmissionState)value;
        if (transmissionState == IRadio::TRANSMISSION_STATE_TRANSMITTING && newRadioTransmissionState == IRadio::TRANSMISSION_STATE_IDLE) {
            handleWithFsm(endTransmission);
            // Volta à recepção ao fim da própria transmissão. Precisa vir depois de
            // handleWithFsm(endTransmission), que agenda endDelay_1.
            turnOnReceiver();
        }
        transmissionState = newRadioTransmissionState;
    }
}

Packet *Lmac1Mac::encapsulate(Packet *msg)
{
    auto frame = makeShared<LoRaMacFrame>();
    frame->setChunkLength(B(headerLength));
    msg->setArrival(msg->getArrivalModuleId(), msg->getArrivalGateId());
    auto tag = msg->getTag<LoRaTag>();

    frame->setTransmitterAddress(address);
    frame->setLoRaTP(tag->getPower().get());
    frame->setLoRaCF(tag->getCenterFrequency());
    frame->setLoRaSF(tag->getSpreadFactor());
    frame->setLoRaBW(tag->getBandwidth());
    frame->setLoRaCR(tag->getCodeRendundance());
    frame->setSequenceNumber(sequenceNumber);
    frame->setReceiverAddress(MacAddress::BROADCAST_ADDRESS);

    ++sequenceNumber;
    frame->setLoRaUseHeader(tag->getUseHeader());

    msg->insertAtFront(frame);

    return msg;
}

Packet *Lmac1Mac::decapsulate(Packet *frame)
{
    auto loraHeader = frame->popAtFront<LoRaMacFrame>();
    frame->addTagIfAbsent<MacAddressInd>()->setSrcAddress(loraHeader->getTransmitterAddress());
    frame->addTagIfAbsent<MacAddressInd>()->setDestAddress(loraHeader->getReceiverAddress());
    frame->addTagIfAbsent<InterfaceInd>()->setInterfaceId(networkInterface->getInterfaceId());

    return frame;
}

/****************************************************************
 * Frame sender functions.
 */
void Lmac1Mac::sendDataFrame(Packet *frameToSend)
{
    EV << "sending Data frame\n";
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    auto frameCopy = frameToSend->dup();

    auto macHeader = frameCopy->peekAtFront<LoRaMacFrame>();

    auto macAddressInd = frameCopy->addTagIfAbsent<MacAddressInd>();
    macAddressInd->setSrcAddress(macHeader->getTransmitterAddress());
    macAddressInd->setDestAddress(macHeader->getReceiverAddress());

    sendDown(frameCopy);
}

void Lmac1Mac::sendAckFrame()
{
    auto frameToAck = static_cast<Packet *>(endSifs->getContextPointer());
    endSifs->setContextPointer(nullptr);
    auto macHeader = makeShared<CsmaCaMacAckHeader>();
    macHeader->setReceiverAddress(MacAddress(frameToAck->peekAtFront<LoRaMacFrame>()->getTransmitterAddress().getInt()));

    EV << "sending Ack frame\n";
    macHeader->setChunkLength(B(ackLength));
    auto frame = new Packet("CsmaAck");
    frame->insertAtFront(macHeader);
    radio->setRadioMode(IRadio::RADIO_MODE_TRANSMITTER);

    auto macAddressInd = frame->addTagIfAbsent<MacAddressInd>();
    macAddressInd->setSrcAddress(macHeader->getTransmitterAddress());
    macAddressInd->setDestAddress(macHeader->getReceiverAddress());

    sendDown(frame);
}

/****************************************************************
 * Helper functions.
 */
void Lmac1Mac::finishCurrentTransmission()
{
    scheduleAt(simTime() + waitDelay1Time, endDelay_1);
    scheduleAt(simTime() + waitDelay1Time + listening1Time, endListening_1);
    scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time, endDelay_2);
    scheduleAt(simTime() + waitDelay1Time + listening1Time + waitDelay2Time + listening2Time, endListening_2);
    deleteCurrentTxFrame();
}

Packet *Lmac1Mac::getCurrentTransmission()
{
    ASSERT(currentTxFrame != nullptr);
    return currentTxFrame;
}

bool Lmac1Mac::isReceiving()
{
    return radio->getReceptionState() == IRadio::RECEPTION_STATE_RECEIVING;
}

bool Lmac1Mac::isAck(const Ptr<const LoRaMacFrame> &frame)
{
    return false;
}

bool Lmac1Mac::isBroadcast(const Ptr<const LoRaMacFrame> &frame)
{
    return frame->getReceiverAddress().isBroadcast();
}

bool Lmac1Mac::isForUs(const Ptr<const LoRaMacFrame> &frame)
{
    return frame->getReceiverAddress() == address;
}

void Lmac1Mac::turnOnReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_RECEIVER);
}

void Lmac1Mac::turnOffReceiver()
{
    LoRaRadio *loraRadio;
    loraRadio = check_and_cast<LoRaRadio *>(radio);
    loraRadio->setRadioMode(IRadio::RADIO_MODE_SLEEP);
}

MacAddress Lmac1Mac::getAddress()
{
    return address;
}

} // namespace flora
