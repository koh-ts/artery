//
// Copyright (C) 2011 Andras Varga
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

#ifndef ARTERY_UDPCAMSENDER_H
#define ARTERY_UDPCAMSENDER_H

#include "inet/common/INETDefs.h"

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "artery/application/CaObject.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/application/Timer.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <iostream>
#include <fstream>


namespace artery {

using namespace inet;
/**
 * UDP application. See NED for more info.
 */
class UDPCamSender : public ApplicationBase
{
  protected:
    enum SelfMsgKinds { START = 1, SEND, STOP };

    // parameters
    std::vector<L3Address> destAddresses;
    int localPort = -1, destPort = -1;
    simtime_t startTime;
    simtime_t stopTime;
    double simInterval;
    const char *packetName = nullptr;
    Timer mTimer;

    int maxHopNum = 0;
    double pcamRange = 0;
    bool fakeCam = false;
    int fakeCamNum = 0;
    double simStartTime = 0;
    double simEndTime = 0;
    int numProxyCamDevsParEdge = 0;
    int numProxyCamDevs = 0;

    double dest_min_dist = 0;
    double dest_max_dist = 1000;

    // state
    UDPSocket socket;
    cMessage *selfMsg = nullptr;

    // statistics
    int numSent = 0;
    int numReceived = 0;

    static simsignal_t sentPkSignal;
    static simsignal_t rcvdPkSignal;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    // chooses random destination address
    virtual L3Address chooseDestAddr();
    virtual void sendPacket();
    virtual std::vector<ApplicationPacket*> searchAndMakeCamPayloads();
    virtual ApplicationPacket* getCamPayload(const VehicleDataProvider*);
    virtual std::vector<ApplicationPacket*> makeFakeCamPayloads();
//    virtual vanetza::asn1::Cam createCooperativeAwarenessMessage();
    virtual void processPacket(cPacket *msg);
    virtual void setSocketOptions();

    virtual void processStart();
    virtual void processSend();
    virtual void processStop();

    virtual bool handleNodeStart(IDoneCallback *doneCallback) override;
    virtual bool handleNodeShutdown(IDoneCallback *doneCallback) override;
    virtual void handleNodeCrash() override;
    std::ofstream ofs;


  public:
    UDPCamSender() {}
    ~UDPCamSender();

};

} // namespace inet

#endif // ifndef __INET_UDPCAMSENDER_H

