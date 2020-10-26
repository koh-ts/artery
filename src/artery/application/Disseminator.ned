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

package artery.application;

import inet.node.inet.StandardHost;
import org.car2x.veins.base.modules.IBaseApplLayer;
import org.car2x.veins.base.modules.IMobility;
import org.car2x.veins.modules.nic.INic80211p;

//
// TODO auto-generated module
//
module Disseminator extends StandardHost
{
    parameters:
        @signal[rcvdPk](type=cPacket);
        //        string interfaceTableModule;   // The path to the InterfaceTable module
        //        int localPort;  // local port to listen on
        @display("i=block/app");
        @display("i=block/wrxtx;is=vs");
        string applType = default("artery.veins.ApplicationLayer"); // type of application layer
        string nicType = default("org.car2x.veins.modules.nic.Nic80211p"); // type of network interface card
//        string mobilityType = default("artery.veins.Mobility");

    gates:
        input radioIN @directIn; // gate for sendDirect

    submodules:
        appl: <applType> like IBaseApplLayer {
            parameters:
                @display("p=60,50");
                middleware.typename = "StationaryMiddleware";
                vanetza.position.typename = "StationaryPositionProvider";
                vanetza.router.isMobile = false;
                **.mobilityModule = absPath("^.^.mobility");
                **.traciCoreModule = "traci.core";
        }

        nic: <nicType> like INic80211p {
            @display("p=110,489");
        }

//        mobility: <mobilityType> like IMobility {
//          parameters:
//            @display("p=130,150");
//            x = 0.0; // overwritten by TraCI
//            y = 0.0; // overwritten by TraCI
//            z = default(1.5);
//            @display("p=130,150");
//        }

    connections:
        nic.upperLayerOut --> appl.lowerLayerIn;
        nic.upperLayerIn <-- appl.lowerLayerOut;
        nic.upperControlOut --> appl.lowerControlIn;
        nic.upperControlIn <-- appl.lowerControlOut;
        radioIN --> nic.radioIn;

}
