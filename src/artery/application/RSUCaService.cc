#include "artery/application/CaObject.h"
#include "artery/application/RSUCaService.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "artery/application/UDPCamListener.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "inet/networklayer/contract/ipv4/IPv4ControlInfo.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include "veins/base/utils/Coord.h"
#include "artery/veins/VeinsMobility.h"
#include <chrono>
namespace artery
{

using namespace omnetpp;


static const simsignal_t scSignalCamReceived = cComponent::registerSignal("CamReceived");
static const simsignal_t scSignalCamSent = cComponent::registerSignal("CamSent");
static const auto scLowFrequencyContainerInterval = std::chrono::milliseconds(500);

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
    boost::units::quantity<U> v { q };
    return std::round(v.value());
}


Define_Module(RSUCaService)

RSUCaService::RSUCaService() :
        mVehicleDataProvider(nullptr),
        mTimer(nullptr),
        mGenCamMin { 100, SIMTIME_MS },
        mGenCamMax { 1000, SIMTIME_MS },
        mGenCam(mGenCamMax),
        mGenCamLowDynamicsCounter(0),
        mGenCamLowDynamicsLimit(3)
{
}

void RSUCaService::initialize()
{
    ItsG5BaseService::initialize();
    mTimer = &getFacilities().get_const<Timer>();

    if (std::strstr(this->getFullPath().c_str(),"GridWorld") != NULL && std::strstr(this->getFullPath().c_str(), "pcam[24]") != NULL) {
      std::string output = par("outputDir");
      output += "output_" + (std::string)this->getFullPath() + ".txt";
      ofs.open(output, std::ios::out);
    } else if (std::strstr(this->getFullPath().c_str(),"BunkyoWorld") != NULL && std::strstr(this->getFullPath().c_str(), "pcam[0]") != NULL) {
      std::string output = par("outputDir");
      output += "output_" + (std::string)this->getFullPath() + ".txt";
      ofs.open(output, std::ios::out);
    }

    findHost()->subscribe(artery::UDPCamListener::rcvdPkSignal,this);

}


void RSUCaService::sendCAMWithPacket(omnetpp::cPacket* pk) {
//  std::cout << "sending cam with packet..." << endl;
  EV_INFO << "sending cam......" << endl;
  // std::cout << "sending cam ........" << endl;
//  ofs << "time: " << omnetpp::simTime() << "\t" << "src cam time: " << mVehicleDataProvider->updated() << endl;

  auto cam = getCamFromPacket(pk);
  using namespace vanetza;
  btp::DataRequestB request;
  request.destination_port = btp::ports::CAM;
  request.gn.its_aid = aid::CA;
  request.gn.transport_type = geonet::TransportType::SHB;
  request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::_1_S, 1 };
  request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
  request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;
  CaObject obj(std::move(cam));
  emit(scSignalCamSent, &obj);

  using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
  std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
  std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
  payload->layer(OsiLayer::Application) = std::move(buffer);

  UDPDataIndication *ctrl = check_and_cast<UDPDataIndication *>(pk->getControlInfo());
  L3Address srcAddress = ctrl->getSrcAddr();
  if (ofs) {
    ofs << "time: " << simTime()
        << "\tserialnum: " << ((ApplicationPacket *)pk) -> getSequenceNumber()
        << "\tfrom: " << srcAddress
        << "\tto: " << L3AddressResolver().resolve(this->getModuleByPath("^.^.^")->getFullPath().c_str())
        << endl;
  }
  this->request(request, std::move(payload));
}

// double RSUCaService::calcDistance(cPacket *pk) {
//   if (pk->hasPar("data")) {
//     std::string s = (std::string)(pk->par("data"));
//     std::stringstream ss{s};
//     std::string buf;
//     int count = 0;
//     double x,y;
//     while (std::getline(ss, buf, ',')) {
//       if(count == 7) {
//         x = std::stod(buf);
//       } else if (count == 8) {
//         y = std::stod(buf);
//       } else if (count >= 9) {
//         break;
//       }
//       count++;
//     }
//     Position src_pos = Position(x,y);

//     Coord pcam_coord = check_and_cast<VeinsMobility *>(this->getModuleByPath("^.^.mobility"))->getCurrentPosition();
//     Position pcam_pos = Position(pcam_coord.x, pcam_coord.y);
//     return boost::geometry::distance(src_pos, pcam_pos);
//   } else {
//     std::cout << "no data found" << endl;
//     return false;
//   }
// }


vanetza::asn1::Cam RSUCaService::getCamFromPacket(omnetpp::cPacket* pk) {
  vanetza::asn1::Cam message;
  std::vector<std::string> pcam;
  double distance;
  if (pk->hasPar("data1")) {
    std::string s = (std::string)(pk->par("data1"));
    std::stringstream ss{s};
    std::string buf;
    int i = 0;
    std::cout << s << endl;
    while (std::getline(ss, buf, ',')) {
      pcam.push_back(buf);
      // std::cout << i << ":" << std::stol(buf) << endl;
      // i++;
    }
  } else {
    std::cout << "no data found" << endl;
  }
  auto itr_pcam = pcam.begin();
  // std::cout << distance << endl;
  ItsPduHeader_t& header = (*message).header;
  header.protocolVersion = std::stol(*itr_pcam++); //0
  header.messageID = std::stol(*itr_pcam++); //1
  header.stationID = (StationID_t)std::stol(*itr_pcam++); //2

  CoopAwareness_t& cam = (*message).cam;
  std::string srcTime = *itr_pcam;
  cam.generationDeltaTime = (uint16_t)std::stol(*itr_pcam++) * GenerationDeltaTime_oneMilliSec; //3
  BasicContainer_t& basic = cam.camParameters.basicContainer;
  HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

  basic.stationType = (StationType_t)std::stol(*itr_pcam++);
  basic.referencePosition.altitude.altitudeValue = (AltitudeValue_t)std::stol(*itr_pcam++); //4
  basic.referencePosition.altitude.altitudeConfidence = (AltitudeConfidence_t)std::stol(*itr_pcam++); //5
  //basic.referencePosition.longitude = (Longitude_t)std::stol(*itr_pcam++);
  itr_pcam++; //6
  //analysisのための便宜上の値変更(udp sequencenum)
  basic.referencePosition.longitude = (Longitude_t)((ApplicationPacket *)pk) -> getSequenceNumber();

  //basic.referencePosition.latitude = (Latitude_t)std::stol(*itr_pcam++);
  itr_pcam++; //7
  //analysisのための便宜上の値変更(udp src time マイクロ秒まで取得したかったのでここに入れる)
  basic.referencePosition.latitude = (Latitude_t)std::stol(srcTime);

  basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = (HeadingValue_t)std::stol(*itr_pcam++);
  basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence = (SemiAxisLength_t)std::stol(*itr_pcam++);
  basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence = (SemiAxisLength_t)std::stol(*itr_pcam++);

  hfc.present = (HighFrequencyContainer_PR)std::stol(*itr_pcam++);
  BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
  bvc.heading.headingValue = (HeadingValue_t)std::stol(*itr_pcam++);
  bvc.heading.headingConfidence = (HeadingConfidence_t)std::stol(*itr_pcam++);
  bvc.speed.speedValue = (SpeedValue_t)std::stol(*itr_pcam++);
  bvc.speed.speedConfidence = (SpeedConfidence_t)std::stol(*itr_pcam++);
  bvc.driveDirection = (DriveDirection_t)std::stol(*itr_pcam++);
  bvc.longitudinalAcceleration.longitudinalAccelerationValue = (LongitudinalAccelerationValue_t)std::stol(*itr_pcam++);
  bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = (AccelerationConfidence_t)std::stol(*itr_pcam++);
  bvc.curvature.curvatureValue = (CurvatureValue_t)std::stol(*itr_pcam++);
  bvc.curvature.curvatureConfidence = (CurvatureConfidence_t)std::stol(*itr_pcam++);
  bvc.curvatureCalculationMode = (CurvatureCalculationMode_t)std::stol(*itr_pcam++);
  bvc.yawRate.yawRateValue = (YawRateValue_t)std::stol(*itr_pcam++);
  bvc.vehicleLength.vehicleLengthValue = (VehicleLengthValue_t)std::stol(*itr_pcam++);
  bvc.vehicleLength.vehicleLengthConfidenceIndication = (VehicleLengthConfidenceIndication_t)std::stol(*itr_pcam++);
  bvc.vehicleWidth = (VehicleWidth_t)std::stol(*itr_pcam);

  std::string error;
  if (!message.validate(error)) {
    throw cRuntimeError("Invalid High Frequency CAM: %s", error.c_str());
  }

  return message;
}

void RSUCaService::receiveSignal(cComponent* source, simsignal_t signal, cObject* obj1, cObject* obj2)
{
    Enter_Method_Silent();
//    std::cout << simTime() << endl;
    if (signal == artery::UDPCamListener::rcvdPkSignal) {
        sendCAMWithPacket((cPacket*)obj1);
    }
}




void RSUCaService::trigger()
{
    checkTriggeringConditions(simTime());
}

void RSUCaService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
    auto microdegree = vanetza::units::degree * boost::units::si::micro;
    auto decidegree = vanetza::units::degree * boost::units::si::deci;
    auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
    auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;
    Asn1PacketVisitor<vanetza::asn1::Cam> visitor;
    const vanetza::asn1::Cam* cam = boost::apply_visitor(visitor, *packet);
    if (cam && cam->validate()) {
        CaObject obj = visitor.shared_wrapper;
        // std::cout << "scSignalCamReceived" << endl;
        emit(scSignalCamReceived, &obj);
        const vanetza::asn1::Cam& msg = obj.asn1();

        static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
        auto tai = mTimer->reconstructMilliseconds(msg->cam.generationDeltaTime);
        const omnetpp::SimTime timeStamp = mTimer->getTimeFor(tai);
        // std::cout << tai << endl;
        // fprintf(fp, "%d\t%s", (int)(msg->header.stationID), std::to_string(expiry));
        // std::cout << "generationDeltaTime is: " << msg->cam.generationDeltaTime;
        if(ofs) {
          ofs << "time: " << omnetpp::simTime()
          << "\tsrc station id: " << msg->header.stationID
          // ofs << "recv pos is: " << mVehicleDataProvider->longitude() << endl;
          << "\tsrc cam time: " << timeStamp
          << "\tsrc pos: " << msg->cam.camParameters.basicContainer.referencePosition.latitude << ","
          << msg->cam.camParameters.basicContainer.referencePosition.longitude
          << "\tdst pos: " << round(mVehicleDataProvider->latitude(), microdegree) * Latitude_oneMicrodegreeNorth << ","
          << round(mVehicleDataProvider->longitude(), microdegree) * Longitude_oneMicrodegreeEast
          << endl;
        }
        // std::cout << "before: " << round(mVehicleDataProvider->latitude(), vanetza::units::degree) << " after: " << round(mVehicleDataProvider->latitude(), microdegree) * Latitude_oneMicrodegreeNorth << endl;
        // std::cout << "before: " << round(mVehicleDataProvider->longitude(), vanetza::units::degree) << " after: " << round(mVehicleDataProvider->longitude(), microdegree) * Longitude_oneMicrodegreeEast << endl;
        // ofs << "\t src pos is: " << msg->cam.camParameters.basicContainer.referencePosition.longitude << "," << msg->cam.camParameters.basicContainer.referencePosition.latitude << endl;
        // ofs << "distance is: " << Position::distance(mVehicleDataProvider->position(),

        mLocalDynamicMap->updateAwareness(obj);
    }
}

void RSUCaService::checkTriggeringConditions(const SimTime& T_now)
{
    // provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
    SimTime& T_GenCam = mGenCam;
    const SimTime& T_GenCamMin = mGenCamMin;
    const SimTime& T_GenCamMax = mGenCamMax;
    const SimTime T_GenCamDcc = mDccRestriction ? genCamDcc() : mGenCamMin;
    const SimTime T_elapsed = T_now - mLastCamTimestamp;

    if (T_elapsed >= T_GenCamDcc) {
        if (mFixedRate) {
            sendCam(T_now);
        } else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
            sendCam(T_now);
            T_GenCam = std::min(T_elapsed, T_GenCamMax); /*< if middleware update interval is too long */
            mGenCamLowDynamicsCounter = 0;
        } else if (T_elapsed >= T_GenCam) {
            sendCam(T_now);
            if (++mGenCamLowDynamicsCounter >= mGenCamLowDynamicsLimit) {
                T_GenCam = T_GenCamMax;
            }
        }
    }
}

bool RSUCaService::checkHeadingDelta() const
{
    return abs(mLastCamHeading - mVehicleDataProvider->heading()) > mHeadingDelta;
}

bool RSUCaService::checkPositionDelta() const
{
    return (distance(mLastCamPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool RSUCaService::checkSpeedDelta() const
{
    return abs(mLastCamSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void RSUCaService::sendCam(const SimTime& T_now)
{
    EV_INFO << "sending cam......" << endl;
    // std::cout << "sending cam ........" << endl;
    if (ofs) {
      ofs << "time: " << omnetpp::simTime() << "\t" << "src cam time: " << mVehicleDataProvider->updated() << endl;
    }
    uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
    auto cam = createCooperativeAwarenessMessage(*mVehicleDataProvider, genDeltaTimeMod);

    mLastCamPosition = mVehicleDataProvider->position();
    mLastCamSpeed = mVehicleDataProvider->speed();
    mLastCamHeading = mVehicleDataProvider->heading();
    mLastCamTimestamp = T_now;
    if (T_now - mLastLowCamTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
        addLowFrequencyContainer(cam);
        mLastLowCamTimestamp = T_now;
    }

    using namespace vanetza;
    btp::DataRequestB request;
    request.destination_port = btp::ports::CAM;
    request.gn.its_aid = aid::CA;
    request.gn.transport_type = geonet::TransportType::SHB;
    request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::_1_S, 1 };
    request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
    request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

    CaObject obj(std::move(cam));
    emit(scSignalCamSent, &obj);

    using CamByteBuffer = convertible::byte_buffer_impl<asn1::Cam>;
    std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
    std::unique_ptr<convertible::byte_buffer> buffer { new CamByteBuffer(obj.shared_ptr()) };
    payload->layer(OsiLayer::Application) = std::move(buffer);
    this->request(request, std::move(payload));
}

SimTime RSUCaService::genCamDcc()
{
    static const vanetza::dcc::TransmissionLite ca_tx(vanetza::dcc::Profile::DP2, 0);
    auto& trc = getFacilities().get_mutable<vanetza::dcc::TransmitRateThrottle>();
    vanetza::Clock::duration delay = trc.delay(ca_tx);
    SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(delay).count(), SIMTIME_MS };
    return std::min(mGenCamMax, std::max(mGenCamMin, dcc));
}

} // namespace artery
