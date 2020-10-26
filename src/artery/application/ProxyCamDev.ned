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

import inet.node.inet.Router;
import artery.application.Disseminator;
import artery.application.Sensor;
import ned.DatarateChannel;
import org.car2x.veins.base.modules.IMobility;
import inet.visualizer.integrated.IntegratedVisualizer;
//import inet.mobility.contract.IMobility;
import inet.node.inet.StandardHost;
import inet.node.ethernet.EtherSwitch;


network ProxyCamDev
{
    parameters:
        string mobilityType = default("artery.veins.Mobility");
//        @networkNode;
    gates:
        inout ethg;

    types:
        channel ethline extends DatarateChannel
        {
            datarate = 10Gbps;
        }

    submodules:

        switch: EtherSwitch {
            @display("p=77,141");
        }

        disseminator: Disseminator {
            @display("p=228,62");
        }

        sensor: Sensor {
            @display("p=228,174");
        }

        mobility: <mobilityType> like IMobility {
            parameters:
                @display("p=96,50");
        }


    connections:
        ethg <--> switch.ethg++;
        sensor.ethg++ <--> ethline <--> switch.ethg++;
        disseminator.ethg++ <--> ethline <--> switch.ethg++;
}
