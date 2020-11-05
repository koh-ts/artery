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

#ifndef ARTERY_UDPCAMLISTENER_H
#define ARTERY_UDPCAMLISTENER_H

#include "inet/common/INETDefs.h"

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"
#include <iostream>
#include <fstream>
#include "artery/application/CaObject.h"

namespace artery {

using namespace inet;
/**
 * UDP application. See NED for more info.
 */
class UDPCamListener : public ApplicationBase
{
  public:
    static simsignal_t rcvdPkSignal;
  protected:
    UDPSocket socket;

  public:
    double calcDistance(cPacket*);
  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;

    virtual void receiveCAM(cPacket *msg);

    virtual bool handleNodeStart(IDoneCallback *doneCallback) override;
    virtual bool handleNodeShutdown(IDoneCallback *doneCallback) override;
    virtual void handleNodeCrash() override;
    bool checkDistance(cPacket*);
    
    std::ofstream ofs;
    double queueRT = 0;
    double dest_min_dist = 0;
    double dest_max_dist = 0;

  public:
    UDPCamListener() {}
    ~UDPCamListener() {}
};

} // namespace inet

#endif // ifndef ARTERY_UDPCAMLISTENER_H

