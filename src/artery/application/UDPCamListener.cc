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

#include "artery/application/UDPCamListener.h"

#include <boost/geometry/algorithms/distance.hpp>
#include "inet/common/ModuleAccess.h"
#include "inet/transportlayer/contract/udp/UDPControlInfo_m.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "inet/networklayer/contract/ipv4/IPv4ControlInfo.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "artery/veins/VeinsMobility.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/base/utils/Coord.h"
#include <string.h>


namespace artery
{

Define_Module(UDPCamListener);

simsignal_t UDPCamListener::rcvdPkSignal = cComponent::registerSignal("campktrcv");

void UDPCamListener::initialize(int stage)
{
    ApplicationBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        // init statistics
      if (std::strstr(this->getFullPath().c_str(),"GridWorld") != NULL && std::strstr(this->getFullPath().c_str(), "pcam[24]") != NULL) {
        std::string output = par("outputDir");
        output += "output_" + (std::string)this->getFullPath() + ".txt";
        ofs.open(output, std::ios::out);
      } else if (std::strstr(this->getFullPath().c_str(),"BunkyoWorld") != NULL && std::strstr(this->getFullPath().c_str(), "pcam[0]") != NULL) {
        std::string output = par("outputDir");
        output += "output_" + (std::string)this->getFullPath() + ".txt";
        ofs.open(output, std::ios::out);
      }
      dest_min_dist = par("dest_min_dist");
      dest_max_dist = par("dest_max_dist");
    }
}

void UDPCamListener::handleMessageWhenUp(cMessage *msg)
{
    if (msg->getKind() == UDP_I_DATA) {
      receiveCAM(PK(msg));
    }
    else {
        throw cRuntimeError("Message received with unexpected message kind = %d", msg->getKind());
    }
}

void UDPCamListener::receiveCAM(cPacket *pk)
{

    // determine its source address/port
    UDPDataIndication *ctrl = check_and_cast<UDPDataIndication *>(pk->getControlInfo());
    L3Address srcAddress = ctrl->getSrcAddr();
//    std::cout << "cam received" << UDPSocket::getReceivedPacketInfo(pk) << " from:" << srcAddress << endl;

    if(ofs) {
      ofs << "time: " << simTime()
          << "\tserialnum: " << ((ApplicationPacket *)pk)->getSequenceNumber()
          << "\tfrom: " << srcAddress
          << "\tto: " << L3AddressResolver().resolve(this->getParentModule()->getFullPath().c_str())
          << "\tttl: " << ctrl->getTtl();
    }


    if (checkDistance(pk)) {
      emit(rcvdPkSignal, pk);
    } else {
      if(ofs)
      ofs << "\t dropped";
    }
    if(ofs)
    ofs << endl;
    //    std::cout << check_and_cast<artery::Disseminator>this->getParentModule()->handleMessage(pk) << endl;

    delete pk;
}


bool UDPCamListener::checkDistance(cPacket *pk) {
  if (pk->hasPar("data")) {
    std::string s = (std::string)(pk->par("data"));
    // std::cout << s << endl;
    std::stringstream ss{s};
    std::string buf;
    int count = 0;
    double x,y;
    while (std::getline(ss, buf, ',')) {
      if(count == 7) {
        x = std::stod(buf);
      } else if (count == 8) {
        y = std::stod(buf);
      } else if (count >= 9) {
        break;
      }
      count++;
    }
    Position src_pos = Position(x,y);

    Coord pcam_coord = check_and_cast<VeinsMobility *>(this->getModuleByPath("^.^.mobility"))->getCurrentPosition();
    Position pcam_pos = Position(pcam_coord.x, pcam_coord.y);
    double distance = boost::geometry::distance(src_pos, pcam_pos);
    std::cout << "packet" << endl;
    std::cout << std::to_string(distance) << endl;
    std::string tmp = "";
    count = 0;
    std::stringstream sss{s};
    while (std::getline(sss, buf, ',')) {
      if (count != 10){
        tmp += buf + ",";
      } else {
        tmp += std::to_string(distance) + ",";
      }
      count++;
    }
    tmp.pop_back();
    pk->addPar("data1") = tmp.c_str();

    double queueRT = check_and_cast<Mac1609_4 *>(this->getModuleByPath("^.nic.mac1609_4"))->getQueueRatio();
//    if (std::strstr(this->getFullPath().c_str(),"pcam[24]") != NULL) {
//      std::cout << "src_pos is: " << x << "," << y << endl;
//      std::cout << "pcam_pos is: " << pcam_coord.x << "," << pcam_coord.y << endl;
//      std::cout << "distance is: " << distance << " queueRT is: " << queueRT << " check is: " << (queueRT < 1 - (distance - dest_min_dist) / (dest_max_dist - dest_min_dist)) << endl;
//    }
    return queueRT < 1 - (distance - dest_min_dist) / (dest_max_dist - dest_min_dist);
  } else {
    std::cout << "no data found" << endl;
    return false;
  }
}

double UDPCamListener::calcDistance(cPacket *pk) {
  if (pk->hasPar("data")) {
    std::string s = (std::string)(pk->par("data"));
    // std::cout << s << endl;
    std::stringstream ss{s};
    std::string buf;
    int count = 0;
    double x,y;
    while (std::getline(ss, buf, ',')) {
      if(count == 7) {
        x = std::stod(buf);
      } else if (count == 8) {
        y = std::stod(buf);
      } else if (count >= 9) {
        break;
      }
      count++;
    }
    Position src_pos = Position(x,y);

    Coord pcam_coord = check_and_cast<VeinsMobility *>(this->getModuleByPath("^.^.mobility"))->getCurrentPosition();
    Position pcam_pos = Position(pcam_coord.x, pcam_coord.y);
    return boost::geometry::distance(src_pos, pcam_pos);
  } else {
    std::cout << "no data found" << endl;
    return false;
  }
}


void UDPCamListener::finish()
{
    ApplicationBase::finish();
}

bool UDPCamListener::handleNodeStart(IDoneCallback *doneCallback)
{
    int localPort = par("localPort");
    socket.setOutputGate(gate("udpOut"));
    socket.bind(localPort);
    return true;
}

bool UDPCamListener::handleNodeShutdown(IDoneCallback *doneCallback)
{
    //TODO if(socket.isOpened()) socket.close();
    return true;
}

void UDPCamListener::handleNodeCrash()
{
}

} // namespace artery
