//
// Copyright (C) 2000 Institut fuer Telematik, Universitaet Karlsruhe
// Copyright (C) 2004,2011 Andras Varga
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

#include "artery/application/UDPCamSender.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <random>

#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/NodeOperations.h"
#include "inet/transportlayer/contract/udp/UDPControlInfo_m.h"
#include "artery/application/VehicleMiddleware.h"
#include "veins/base/utils/MiXiMDefs.h"
//#include "artery/utility/Geometry.h"
#include "artery/traci/VehicleController.h"
#include "artery/veins/VeinsMobility.h"

namespace artery {

using namespace omnetpp;

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
  boost::units::quantity<U> v { q };
  return std::round(v.value());
}

Define_Module(UDPCamSender);

simsignal_t UDPCamSender::sentPkSignal = registerSignal("sentPk");
simsignal_t UDPCamSender::rcvdPkSignal = registerSignal("rcvdPk");


UDPCamSender::~UDPCamSender()
{
    cancelAndDelete(selfMsg);
}

void UDPCamSender::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        numSent = 0;
        numReceived = 0;
        WATCH(numSent);
        WATCH(numReceived);

        localPort = par("localPort");
        destPort = par("destPort");

        std::random_device rnd;     // 非決定的な乱数生成器を生成
        std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
        std::uniform_real_distribution<> real_rand(0, par("sendInterval").doubleValue());        // [0, 1] 範囲の一様乱数
        startTime = simTime() + uniform(0,real_rand(mt));
        simInterval = par("simInterval").doubleValue();
        simStartTime = par("simStartTime");
        simEndTime = par("simEndTime");
        stopTime = simStartTime + simInterval;
        numProxyCamDevsParEdge = par("numProxyCamDevsParEdge"); //gridworldの時
        numProxyCamDevs = par("numProxyCamDevs");  //bunkyoworldの時
        dest_min_dist = par("dest_min_dist");
        dest_max_dist = par("dest_max_dist");
        packetName = par("packetName");
        maxHopNum = par("maxHopNum");
        pcamRange = par("pcamRange");
        fakeCam = par("fakeCam");
        fakeCamNum = par("fakeCamNum");
        if (stopTime >= SIMTIME_ZERO && stopTime < startTime)
            throw cRuntimeError("Invalid startTime/stopTime parameters");

        std::string output = par("outputDir");
        output += "output_" + this->getFullPath() + "_sender.txt";
        ofs.open(output, std::ios::out);
        mTimer.setTimebase(par("datetime"));
        selfMsg = new cMessage("sendTimer");
    }
}

void UDPCamSender::finish()
{
    recordScalar("packets sent", numSent);
    recordScalar("packets received", numReceived);
    ApplicationBase::finish();
}

void UDPCamSender::setSocketOptions()
{
    int timeToLive = par("timeToLive");
    if (timeToLive != -1)
        socket.setTimeToLive(timeToLive);

    int typeOfService = par("typeOfService");
    if (typeOfService != -1)
        socket.setTypeOfService(typeOfService);

    const char *multicastInterface = par("multicastInterface");
    if (multicastInterface[0]) {
        IInterfaceTable *ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
        InterfaceEntry *ie = ift->getInterfaceByName(multicastInterface);
        if (!ie)
            throw cRuntimeError("Wrong multicastInterface setting: no interface named \"%s\"", multicastInterface);
        socket.setMulticastOutputInterface(ie->getInterfaceId());
    }

    bool receiveBroadcast = par("receiveBroadcast");
    if (receiveBroadcast)
        socket.setBroadcast(true);

    bool joinLocalMulticastGroups = par("joinLocalMulticastGroups");
    if (joinLocalMulticastGroups) {
        MulticastGroupList mgl = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this)->collectMulticastGroups();
        socket.joinLocalMulticastGroups(mgl);
    }
}

L3Address UDPCamSender::chooseDestAddr()
{
    int k = intrand(destAddresses.size());
    if (destAddresses[k].isLinkLocal()) {    // KLUDGE for IPv6
        const char *destAddrs = par("destAddresses");
        cStringTokenizer tokenizer(destAddrs);
        const char *token = nullptr;

        for (int i = 0; i <= k; ++i)
            token = tokenizer.nextToken();
        destAddresses[k] = L3AddressResolver().resolve(token);
    }
    return destAddresses[k];
}

void UDPCamSender::sendPacket()
{
    std::vector<ApplicationPacket*> payloads = searchAndMakeCamPayloads();

//    L3Address destAddr = chooseDestAddr();

    for (auto it = payloads.begin();it != payloads.end(); ++it) {
      for (int i = 0 ; i < destAddresses.size(); i++) {
        emit(sentPkSignal, *it);
        socket.sendTo((*it)->dup(), destAddresses[i], destPort);
        ofs << "time: " << simTime()
            << "\tserial num: " << (*it)->getSequenceNumber()
            << "\tfrom: " << L3AddressResolver().resolve(this->getParentModule()->getFullPath().c_str())
            << "\tto: " << destAddresses[i]
            << endl;
        // std::cout << "udp packet sent" << endl;
      }
      delete (*it);
    }
}

std::vector<ApplicationPacket*> UDPCamSender::searchAndMakeCamPayloads() {
  EV_INFO << "sending cam......" << endl;

  if (fakeCam) {
    if (simTime() > simStartTime) {
      return makeFakeCamPayloads();
    } else {
      std::vector<ApplicationPacket *> payloads;
      return payloads;
    }
  }

  std::vector<const VehicleDataProvider *> vdps;

  VeinsMobility* mobility = check_and_cast<VeinsMobility *>(this->getParentModule()->getParentModule()->getModuleByPath(".mobility"));
  Coord cpos = mobility->getCurrentPosition();
  //  traci::VehicleController* controller = (traci::VehicleController*)(mobility->getVehicleController());
//  auto pos = controller->getPosition();
  auto pos = Position(cpos.x, cpos.y);

  auto mod = getSimulation()->getSystemModule();

  for (cModule::SubmoduleIterator iter(mod); !iter.end(); iter++) {
    cModule* submod = SUBMODULE_ITERATOR_TO_MODULE(iter);
    if (strstr(submod->getName(),"node")!=NULL) {
      VehicleMiddleware* middleware = check_and_cast<VehicleMiddleware *>(submod->getModuleByPath(".appl.middleware"));
      const VehicleDataProvider* vdp = &middleware->getFacilities().get_const<VehicleDataProvider>();
      // std::cout << "distance is " << (double)boost::geometry::distance(vdp->position(), pos) << endl;
      if (boost::geometry::distance(vdp->position(), pos) < pcamRange) {
        vdps.push_back(vdp);
      }
    }
  }

  std::vector<ApplicationPacket *> payloads;
  for (auto it = vdps.begin();it != vdps.end(); ++it) {
    payloads.push_back(getCamPayload(*it));
  }

  return payloads;
}

ApplicationPacket* UDPCamSender::getCamPayload(const VehicleDataProvider* vdp) {
  auto microdegree = vanetza::units::degree * boost::units::si::micro;
  auto decidegree = vanetza::units::degree * boost::units::si::deci;
  auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
  auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

  std::ostringstream str;
  str << "1" << ","                             //header.protocolVersion
      << ItsPduHeader__messageID_cam << ","     //header.messageId
      << "0" << ","                             //header.stationID
      << countTaiMilliseconds(mTimer.getTimeFor(simTime())) << ","   //cam.generationDeltaTime
      << StationType_passengerCar << ","
      << AltitudeValue_unavailable << ","
      << AltitudeConfidence_unavailable << ","
      << round(vdp->longitude(), microdegree) * Longitude_oneMicrodegreeEast << ","
      << round(vdp->latitude(), microdegree) * Latitude_oneMicrodegreeNorth << ","
      << HeadingValue_unavailable << ","
      << SemiAxisLength_unavailable << ","
      << SemiAxisLength_unavailable << ","
      << HighFrequencyContainer_PR_basicVehicleContainerHighFrequency << ","
      << round(vdp->heading(), decidegree) << ","
      << HeadingConfidence_equalOrWithinOneDegree << ","
      << round(vdp->speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec << ","
      << SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3 << ",";

  if (vdp->speed().value() >= 0.0) {
    str << DriveDirection_forward << ",";
  } else {
    str << DriveDirection_backward << ",";
  }

  const double lonAccelValue = vdp->acceleration() / vanetza::units::si::meter_per_second_squared;
  // extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
  if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
    str << lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward << ",";
  } else {
    str << LongitudinalAccelerationValue_unavailable << ",";
  }

  str << AccelerationConfidence_unavailable << ",";
  if (abs(vdp->curvature() / vanetza::units::reciprocal_metre) * 10000.0 >= 1023) {
    str << 1023 << ",";
  } else {
    str << abs(vdp->curvature() / vanetza::units::reciprocal_metre) * 10000.0 << ",";
  }

  str << CurvatureConfidence_unavailable << ","
      << CurvatureCalculationMode_yawRateUsed << ",";
  if (abs(round(vdp->yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0) >= YawRateValue_unavailable) {
    str << YawRateValue_unavailable << ",";
  } else {
    str << round(vdp->yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0 << ",";
  }

  str << VehicleLengthValue_unavailable << ","
      << VehicleLengthConfidenceIndication_noTrailerPresent << ","
      << VehicleWidth_unavailable << endl;

  ApplicationPacket *payload = new ApplicationPacket("CamPacket");
  payload->setByteLength(sizeof(str));
  payload->setSequenceNumber(numSent);
  numSent++;
  payload->addPar("data") = str.str().c_str();
  return payload;
}

std::vector<ApplicationPacket*> UDPCamSender::makeFakeCamPayloads() {
  auto microdegree = vanetza::units::degree * boost::units::si::micro;
  auto decidegree = vanetza::units::degree * boost::units::si::deci;
  auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
  auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

  int fPos = ((std::string)this->getFullPath()).find("[");
  int lPos = ((std::string)this->getFullPath()).find("]");
  int pcamnum = std::stoi(((std::string)this->getFullPath()).substr(fPos + 1, lPos-fPos - 1));

  long double st = (long double) omnetpp::simTime().dbl();

  // make random position
  VeinsMobility* mobility = check_and_cast<VeinsMobility *>(this->getParentModule()->getParentModule()->getModuleByPath(".mobility"));
  Coord pcam_pos = mobility->getCurrentPosition();

  std::random_device rnd1;     // 非決定的な乱数生成器を生成
  std::mt19937 mt1(rnd1());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
  std::uniform_real_distribution<> real_rand(0, 1);        // [0, 1] 範囲の一様乱数

  std::random_device rnd2;     // 非決定的な乱数生成器を生成
      std::mt19937 mt2(rnd2());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
      std::uniform_real_distribution<> pos_rand(0, pcamRange*2);        // [0, pcamRange*2] 範囲の一様乱数 あとでpcamRange分マイナスする

  // 十字路の中心にPCAM装置があり、道路上に車両がいなければならないので、車両の(x,y)はPCAM装置の(X,Y) + (0, 0~pcamRange) or (0~pcamRange,0)

  

  

  std::vector<ApplicationPacket *> payloads;
  for (int i = 0; i < fakeCamNum; i++) {
    pcam_pos = mobility->getCurrentPosition();
    if (real_rand(mt1) > 0.5) { //ここから
    pcam_pos += Coord(0,pos_rand(mt2) - pcamRange);
  } else {
    pcam_pos += Coord(pos_rand(mt2) - pcamRange, 0);
  }
    std::ostringstream str;
    str << "1" << ","                             //header.protocolVersion
      << ItsPduHeader__messageID_cam << ","     //header.messageId
      << pcamnum << ","                         //header.stationID
//      << countTaiMilliseconds(mTimer.getTimeFor(simTime())) << ","   //cam.generationDeltaTime
      << (long int)(st * 1000000)<< ","   //cam.generationDeltaTime
      << StationType_passengerCar << ","
      << AltitudeValue_unavailable << ","
      << AltitudeConfidence_unavailable << ","
      << pcam_pos.x << ","
      << pcam_pos.y << ","
      << HeadingValue_unavailable << ","
      << SemiAxisLength_unavailable << ","
      << SemiAxisLength_unavailable << ","
      << HighFrequencyContainer_PR_basicVehicleContainerHighFrequency << ","
      << 0 << ","
      << HeadingConfidence_equalOrWithinOneDegree << ","
      << 0 << ","
      << SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3 << ",";
  str << DriveDirection_forward << ",";
  str << LongitudinalAccelerationValue_unavailable << ",";
  str << AccelerationConfidence_unavailable << ",";
  str << 1023 << ",";
  str << CurvatureConfidence_unavailable << ","
      << CurvatureCalculationMode_yawRateUsed << ",";
  str << YawRateValue_unavailable << ",";
  str << VehicleLengthValue_unavailable << ","
      << VehicleLengthConfidenceIndication_noTrailerPresent << ","
      << VehicleWidth_unavailable << endl;
//ここまでfor文に入れる
//  std::cout << str.str() << endl;
    ApplicationPacket *payload = new ApplicationPacket("CamPacket");
    payload->setByteLength(sizeof(str));
    payload->addPar("data") = str.str().c_str();

    ApplicationPacket *p = payload->dup();
    p->setSequenceNumber(numSent);
    payloads.push_back(p);
    numSent++;
    delete payload;
  }

  

  return payloads;
}

//vanetza::asn1::Cam

void UDPCamSender::processStart()
{
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), localPort);
    setSocketOptions();

    const char *destAddrs = par("destAddresses");
    if (strstr(destAddrs,"all")!= NULL) {
      if (strstr(this->getFullPath().c_str(), "GridWorld") != NULL) {
        // int num = par("numProxyCamDevsParEdge");
        // int fPos = ((std::string)this->getFullPath()).find("[");
        // int lPos = ((std::string)this->getFullPath()).find("]");
        // int pcamnum = std::stoi(((std::string)this->getFullPath()).substr(fPos + 1, lPos-fPos - 1));
        // int row = (pcamnum - pcamnum % num) / num;
        // int col = pcamnum % num;
        //
        // for (int hop = 0; hop <= maxHopNum; hop++) {
        //   for (int row_hop = 0; row_hop <= hop; row_hop++) {
        //     int col_hop = hop - row_hop;
        //     if (row - row_hop >= 0 && col - col_hop >= 0) {
        //       L3Address result;
        //       L3AddressResolver().tryResolve(
        //           ((std::string)"pcam[" + std::to_string((row - row_hop) * num + (col - col_hop)) + (std::string)"].disseminator").c_str()
        //           , result);
        //       if (result.isUnspecified())
        //         std::cout << "resolve address error" << endl;
        //       else
        //         destAddresses.push_back(result);
        //     }
        //     if (row - row_hop >= 0 && col + col_hop < num) {
        //       L3Address result;
        //       L3AddressResolver().tryResolve(
        //           ((std::string)"pcam[" + std::to_string((row - row_hop) * num + (col + col_hop)) + (std::string)"].disseminator").c_str()
        //           , result);
        //       if (result.isUnspecified())
        //         std::cout << "resolve address error" << endl;
        //       else
        //         destAddresses.push_back(result);
        //     }
        //     if (row + row_hop < num && col - col_hop >= 0 ) {
        //       L3Address result;
        //       L3AddressResolver().tryResolve(
        //           ((std::string)"pcam[" + std::to_string((row + row_hop) * num + (col - col_hop)) + (std::string)"].disseminator").c_str()
        //           , result);
        //       if (result.isUnspecified())
        //         std::cout << "resolve address error" << endl;
        //       else
        //         destAddresses.push_back(result);
        //     }
        //     if (row + row_hop < num && col + col_hop < num) {
        //       L3Address result;
        //       L3AddressResolver().tryResolve(
        //           ((std::string)"pcam[" + std::to_string((row + row_hop) * num + (col + col_hop)) + (std::string)"].disseminator").c_str()
        //           , result);
        //       if (result.isUnspecified())
        //         std::cout << "resolve address error" << endl;
        //       else
        //         destAddresses.push_back(result);
        //     }
        //   }
        // }
        //
        // // 重複している要素を削除
        // sort(destAddresses.begin(), destAddresses.end());
        // destAddresses.erase(unique(destAddresses.begin(), destAddresses.end()), destAddresses.end());
        for (int i = 0; i < numProxyCamDevsParEdge * numProxyCamDevsParEdge; i++) {
          VeinsMobility* mobility = check_and_cast<VeinsMobility *>(getSimulation()->getSystemModule()->getModuleByPath(("pcam[" + std::to_string(i) + "].mobility").c_str()));
          Coord dst_coord = mobility->getCurrentPosition();
          Position dst_pos = Position(dst_coord.x, dst_coord.y);
          Coord src_coord = check_and_cast<VeinsMobility *>(this->getModuleByPath("^.^.mobility"))->getCurrentPosition();
          Position src_pos = Position(src_coord.x, src_coord.y);
          double distance = boost::geometry::distance(src_pos, dst_pos);
          if (distance > dest_min_dist && distance < dest_max_dist) {
            L3Address result;
            L3AddressResolver().tryResolve(
                ((std::string)"pcam[" + std::to_string(i) + (std::string)"].disseminator").c_str()
                , result);
            if (result.isUnspecified())
              std::cout << "resolve address error" << endl;
            else
              destAddresses.push_back(result);
          }
        }
        L3Address result;
        L3AddressResolver().tryResolve(
            this->getModuleByPath("^.^.disseminator")->getFullPath().c_str()
            , result);
        if (result.isUnspecified())
          std::cout << "resolve address error" << endl;
        else
          destAddresses.push_back(result);
        sort(destAddresses.begin(), destAddresses.end());
        destAddresses.erase(unique(destAddresses.begin(), destAddresses.end()), destAddresses.end());
      } else {
      //   // router同士の接続の関係router[*]の*の値で0番目から示している
      //   int connections[][4] = {
      //     {13,22,27,31},
      //     {2,36,22,-1},
      //     {7,3,46,1},
      //     {8,4,5,2},
      //     {5,47,3,-1},
      //     {6,9,3,4},
      //     {10,5,-1,-1},
      //     {11,8,22,2},
      //     {12,9,3,7},
      //     {10,5,8,-1},
      //     {16,6,9,-1},
      //     {13,12,7,-1},
      //     {14,8,11,-1},
      //     {14,0,11,-1},
      //     {15,12,13,-1},
      //     {18,14,-1,-1},
      //     {17,10,-1,-1},
      //     {18,16,-1,-1},
      //     {39,15,17,-1},
      //     {23,20,31,26},
      //     {48,19,-1,-1},
      //     {40,48,-1,-1},
      //     {7,1,33,0},
      //     {19,-1,-1,-1},
      //     {25,30,-1,-1},
      //     {26,24,-1,-1},
      //     {19,25,-1,-1},
      //     {28,33,0,-1},
      //     {29,34,27,-1},
      //     {30,32,28,-1},
      //     {32,24,29,-1},
      //     {0,19,-1,-1},
      //     {38,29,30,-1},
      //     {34,36,22,27},
      //     {35,37,28,33},
      //     {38,37,34,-1},
      //     {37,1,33,-1},
      //     {34,35,36,-1},
      //     {45,32,35,-1},
      //     {18,44,-1,-1},
      //     {41,21,-1,-1},
      //     {42,40,-1,-1},
      //     {43,41,-1,-1},
      //     {44,42,-1,-1},
      //     {39,43,-1,-1},
      //     {46,38,-1,-1},
      //     {47,2,45,-1},
      //     {4,46,-1,-1},
      //     {21,20,-1,-1}
      //   };
      //
      //   int fPos = ((std::string)this->getFullPath()).find("[");
      //   int lPos = ((std::string)this->getFullPath()).find("]");
      //   int pcamnum = std::stoi(((std::string)this->getFullPath()).substr(fPos + 1, lPos-fPos - 1));
      //
      //   std::vector<int> dstPcamNums;
      //   dstPcamNums.push_back(pcamnum);
      //   for (int i = 0;i < maxHopNum; i++) {
      //     std::vector<int> newDstPcamNums;
      //     for(auto itr = dstPcamNums.begin(); itr != dstPcamNums.end(); ++itr) {
      //       newDstPcamNums.push_back(*itr);
      //       for (int j = 0;j < 4;j++) {
      //         if(connections[*itr][j] == -1)
      //           break;
      //         else {
      //           newDstPcamNums.push_back(connections[*itr][j]);
      //         }
      //       }
      //     }
      //     sort(newDstPcamNums.begin(), newDstPcamNums.end());
      //     newDstPcamNums.erase(unique(newDstPcamNums.begin(), newDstPcamNums.end()), newDstPcamNums.end());
      //     dstPcamNums = newDstPcamNums;
      //   }
      //
      //   for(auto itr = dstPcamNums.begin(); itr != dstPcamNums.end(); ++itr) {
      //     cStringTokenizer tokenizer(("10.0." + std::to_string(*itr) + ".3").c_str());
      //     const char *token;
      //
      //     while ((token = tokenizer.nextToken()) != nullptr) {
      //       L3Address result;
      //       L3AddressResolver().tryResolve(token, result);
      //       destAddresses.push_back(result);
      //     }
      //   }
      //   sort(destAddresses.begin(), destAddresses.end());
      //   destAddresses.erase(unique(destAddresses.begin(), destAddresses.end()), destAddresses.end());
      // }
      // // std::cout << "this is " << this->getFullPath() << endl;
      // // for (auto itr = destAddresses.begin();itr != destAddresses.end(); ++itr) {
      // // std::cout << *itr << endl;
      // // }
        for (int i = 0; i < numProxyCamDevs; i++) {
          VeinsMobility* mobility = check_and_cast<VeinsMobility *>(getSimulation()->getSystemModule()->getModuleByPath(("pcam[" + std::to_string(i) + "].mobility").c_str()));
          Coord dst_coord = mobility->getCurrentPosition();
          Position dst_pos = Position(dst_coord.x, dst_coord.y);
          Coord src_coord = check_and_cast<VeinsMobility *>(this->getModuleByPath("^.^.mobility"))->getCurrentPosition();
          Position src_pos = Position(src_coord.x, src_coord.y);
          double distance = boost::geometry::distance(src_pos, dst_pos);
          if (distance > dest_min_dist && distance < dest_max_dist) {
            L3Address result;
            L3AddressResolver().tryResolve(
                ((std::string)"pcam[" + std::to_string(i) + (std::string)"].disseminator").c_str()
                , result);
            if (result.isUnspecified())
              std::cout << "resolve address error" << endl;
            else
              destAddresses.push_back(result);
          }
        }
        L3Address result;
        L3AddressResolver().tryResolve(
            this->getModuleByPath("^.^.disseminator")->getFullPath().c_str()
            , result);
        if (result.isUnspecified())
          std::cout << "resolve address error" << endl;
        else
          destAddresses.push_back(result);
        sort(destAddresses.begin(), destAddresses.end());
        destAddresses.erase(unique(destAddresses.begin(), destAddresses.end()), destAddresses.end());
        // std::cout << "this is " << this->getFullPath() << endl;
        // for (auto itr = destAddresses.begin();itr != destAddresses.end(); ++itr) {
        //   std::cout << *itr << endl;
        // }
      }
    } else {
      cStringTokenizer tokenizer(destAddrs);
      const char *token;

      while ((token = tokenizer.nextToken()) != nullptr) {
          L3Address result;
          L3AddressResolver().tryResolve(token, result);
          if (result.isUnspecified())
              EV_ERROR << "cannot resolve destination address: " << token << endl;
          else
              destAddresses.push_back(result);
      }
    }

    if (!destAddresses.empty()) {
        selfMsg->setKind(SEND);
        processSend();
    }
    else {
        if (stopTime >= SIMTIME_ZERO) {
            selfMsg->setKind(STOP);
            scheduleAt(stopTime, selfMsg);
        }
    }
}

void UDPCamSender::processSend()
{
    sendPacket();
    simtime_t d = simTime() + par("sendInterval").doubleValue();
    if (stopTime < SIMTIME_ZERO || d < stopTime) {
        selfMsg->setKind(SEND);
        scheduleAt(d, selfMsg);
    }
    else {
        selfMsg->setKind(STOP);
        scheduleAt(stopTime, selfMsg);
    }
}

void UDPCamSender::processStop()
{
    socket.close();
    if (stopTime < simEndTime) {
      std::random_device rnd;     // 非決定的な乱数生成器を生成
      std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
      std::uniform_real_distribution<> real_rand(0, par("sendInterval").doubleValue());        // [0, 1] 範囲の一様乱数
      startTime = stopTime + 5 + uniform(0,real_rand(mt));
      stopTime += 15;
      selfMsg->setKind(START);
      scheduleAt(startTime, selfMsg);
    }
}

void UDPCamSender::handleMessageWhenUp(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        ASSERT(msg == selfMsg);
        switch (selfMsg->getKind()) {
            case START:
                processStart();
                break;

            case SEND:
                processSend();
                break;

            case STOP:
                processStop();
                break;

            default:
                throw cRuntimeError("Invalid kind %d in self message", (int)selfMsg->getKind());
        }
    }
    else if (msg->getKind() == UDP_I_DATA) {
        // process incoming packet
        processPacket(PK(msg));
    }
    else if (msg->getKind() == UDP_I_ERROR) {
        EV_WARN << "Ignoring UDP error report\n";
        delete msg;
    }
    else {
        throw cRuntimeError("Unrecognized message (%s)%s", msg->getClassName(), msg->getName());
    }
}

void UDPCamSender::refreshDisplay() const
{
    char buf[100];
    sprintf(buf, "rcvd: %d pks\nsent: %d pks", numReceived, numSent);
    getDisplayString().setTagArg("t", 0, buf);
}

void UDPCamSender::processPacket(cPacket *pk)
{
    emit(rcvdPkSignal, pk);
    EV_INFO << "Received packet: " << UDPSocket::getReceivedPacketInfo(pk) << endl;
    delete pk;
    numReceived++;
}

bool UDPCamSender::handleNodeStart(IDoneCallback *doneCallback)
{
    simtime_t start = std::max(startTime, simTime());
    if ((stopTime < SIMTIME_ZERO) || (start < stopTime) || (start == stopTime && startTime == stopTime)) {
        selfMsg->setKind(START);
        scheduleAt(start, selfMsg);
    }
    return true;
}

bool UDPCamSender::handleNodeShutdown(IDoneCallback *doneCallback)
{
    if (selfMsg)
        cancelEvent(selfMsg);
    //TODO if(socket.isOpened()) socket.close();
    return true;
}

void UDPCamSender::handleNodeCrash()
{
    if (selfMsg)
        cancelEvent(selfMsg);
}

} // namespace inet
