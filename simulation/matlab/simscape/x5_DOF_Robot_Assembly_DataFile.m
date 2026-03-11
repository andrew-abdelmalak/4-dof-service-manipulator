    % Simscape(TM) Multibody(TM) version: 23.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(31).translation = [0.0 0.0 0.0];
smiData.RigidTransform(31).angle = 90.0;
smiData.RigidTransform(31).axis = [0.0 0.0 0.0];
smiData.RigidTransform(31).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [-0.010956254114603377 0.029992190398476299 0.014992441714184943];  % m
smiData.RigidTransform(1).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(1).axis = [1 -4.868750899410934e-31 -4.3853809472693691e-15];
smiData.RigidTransform(1).ID = "B[RotatingWasteLink-2:-:Arm_1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [0.070000000000000034 0.0025000000000002746 1.3877787807814457e-17];  % m
smiData.RigidTransform(2).angle = 2.0943951023931962;  % rad
smiData.RigidTransform(2).axis = [0.57735026918962606 -0.57735026918962595 0.5773502691896254];
smiData.RigidTransform(2).ID = "F[RotatingWasteLink-2:-:Arm_1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [-0.070000000000000007 -0.035355339059327404 0];  % m
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(3).ID = "B[Arm_1-1:-:Arm_2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-0.046461489483465998 -0.0042991010175530628 -0.026811526869617207];  % m
smiData.RigidTransform(4).angle = 1.03673669914961e-14;  % rad
smiData.RigidTransform(4).axis = [-0.99936877587949413 0.035525340211200324 -1.8403587847381554e-16];
smiData.RigidTransform(4).ID = "F[Arm_1-1:-:Arm_2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0.056538510516534191 -0.0042991010175538989 -0.14592550264843204];  % m
smiData.RigidTransform(5).angle = 2.0943951023931993;  % rad
smiData.RigidTransform(5).axis = [0.57735026918962318 0.57735026918962706 0.57735026918962695];
smiData.RigidTransform(5).ID = "B[Arm_2-1:-:Arm_3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-0.0023055609299223617 0.15053772958567579 -0.0056318323359050787];  % m
smiData.RigidTransform(6).angle = 3.1415926535897829;  % rad
smiData.RigidTransform(6).axis = [-7.1951008563724617e-17 7.4106912555278724e-17 -1];
smiData.RigidTransform(6).ID = "F[Arm_2-1:-:Arm_3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [-0.0023055352111781722 -0.0072775419076846412 0.024268167664094786];  % m
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(7).ID = "B[Arm_3-1:-:Gripper-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [-0.037421031639325836 -0.00045636825944249393 0.063576978485936478];  % m
smiData.RigidTransform(8).angle = 3.3371679465407785e-16;  % rad
smiData.RigidTransform(8).axis = [0.74983785536509262 0.66162163708684618 8.2779944199211909e-17];
smiData.RigidTransform(8).ID = "F[Arm_3-1:-:Gripper-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [-0.022896164824373955 0.0067265600567337179 0.049496699518295093];  % m
smiData.RigidTransform(9).angle = 3.1415926535897896;  % rad
smiData.RigidTransform(9).axis = [-0 -1 -0];
smiData.RigidTransform(9).ID = "B[BaseLink-1:-:RotatingWasteLink-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [-0.010956254114603329 -0.013557809601523932 0.014992441714184686];  % m
smiData.RigidTransform(10).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(10).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(10).ID = "F[BaseLink-1:-:RotatingWasteLink-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [-0.010956254114603331 -0.013557809601523696 0.014992441714184686];  % m
smiData.RigidTransform(11).angle = 0;  % rad
smiData.RigidTransform(11).axis = [0 0 0];
smiData.RigidTransform(11).ID = "AssemblyGround[RotatingWasteLink-2:Rotating Waste-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [-0.010956254114603792 0.019742190398476297 -0.032407558285815055];  % m
smiData.RigidTransform(12).angle = 2.0943951023931904;  % rad
smiData.RigidTransform(12).axis = [0.57735026918962418 -0.57735026918962418 -0.57735026918962917];
smiData.RigidTransform(12).ID = "AssemblyGround[RotatingWasteLink-2:Servo MG996R-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [-0.022896164824373955 -0.018273439943266197 0.024496699518295095];  % m
smiData.RigidTransform(13).angle = 0;  % rad
smiData.RigidTransform(13).axis = [0 0 0];
smiData.RigidTransform(13).ID = "AssemblyGround[BaseLink-1:Circular Base-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [-0.012646164824373959 0.0067265600567338844 0.0020966995182950948];  % m
smiData.RigidTransform(14).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(14).axis = [1 0 0];
smiData.RigidTransform(14).ID = "AssemblyGround[BaseLink-1:Servo MG996R-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [-0.037421031639325836 -0.00045636825944234127 0.056876978485936418];  % m
smiData.RigidTransform(15).angle = 1.5707963267948966;  % rad
smiData.RigidTransform(15).axis = [0 0 1];
smiData.RigidTransform(15).ID = "AssemblyGround[Gripper-1:Base-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [-0.053555758979750734 0.049743631740557626 0.059826978485936405];  % m
smiData.RigidTransform(16).angle = 2.6655257904888581;  % rad
smiData.RigidTransform(16).axis = [-0.68597705894379157 -0.68597705894379146 -0.24263336375208586];
smiData.RigidTransform(16).ID = "AssemblyGround[Gripper-1:Link-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(17).translation = [-0.04955575897975073 0.046915835832149422 0.084664010411982238];  % m
smiData.RigidTransform(17).angle = 1.1917508316917051;  % rad
smiData.RigidTransform(17).axis = [1 0 0];
smiData.RigidTransform(17).ID = "AssemblyGround[Gripper-1:Grip-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(18).translation = [-0.051905758979750742 0.029743631740745236 0.040926978485935753];  % m
smiData.RigidTransform(18).angle = 2.5294835689275992;  % rad
smiData.RigidTransform(18).axis = [-0.67087813717661782 0.31598267376054201 -0.67087813717661871];
smiData.RigidTransform(18).ID = "AssemblyGround[Gripper-1:Microservo horn-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(19).translation = [-0.021005758979750724 0.029743631740744292 0.046026978485936412];  % m
smiData.RigidTransform(19).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(19).axis = [0.57735026918962573 -0.57735026918962573 0.57735026918962562];
smiData.RigidTransform(19).ID = "AssemblyGround[Gripper-1:Servo Micro-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(20).translation = [-0.051555758979750725 0.037522217325673901 0.074279246777816388];  % m
smiData.RigidTransform(20).angle = 1.6862155916680708;  % rad
smiData.RigidTransform(20).axis = [0.32135688057509565 -0.32135688057509598 0.8907634425671539];
smiData.RigidTransform(20).ID = "AssemblyGround[Gripper-1:Gear Arm 1-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(21).translation = [-0.051905758979750763 0.021178510782728033 0.046291385516531648];  % m
smiData.RigidTransform(21).angle = 2.7485546625723347;  % rad
smiData.RigidTransform(21).axis = [0.69295156979754791 -0.69295156979754924 0.19908853264371207];
smiData.RigidTransform(21).ID = "AssemblyGround[Gripper-1:Gear Arm 2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(22).translation = [-0.041055758979750723 0.046876253614055396 0.023945876702574941];  % m
smiData.RigidTransform(22).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(22).axis = [-1.4643144908186681e-16 -0.83581572525508618 0.54901008498598125];
smiData.RigidTransform(22).ID = "AssemblyGround[Gripper-1:Grip-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(23).translation = [-0.041055758979750723 0.073641181047058968 0.030080912114543459];  % m
smiData.RigidTransform(23).angle = 1.6856040266847538;  % rad
smiData.RigidTransform(23).axis = [-0.32059547542984818 -0.32059547542984829 -0.89131200051823567];
smiData.RigidTransform(23).ID = "AssemblyGround[Gripper-1:Link-4]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(24).translation = [-0.037055758979750719 0.049743631740557681 0.059826978485936419];  % m
smiData.RigidTransform(24).angle = 1.6822224593735964;  % rad
smiData.RigidTransform(24).axis = [0.31633607653491608 -0.31633607653491608 0.89435058750189877];
smiData.RigidTransform(24).ID = "AssemblyGround[Gripper-1:Link-3]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(25).translation = [-0.049555758979750723 0.049743631740559013 0.049826978485935043];  % m
smiData.RigidTransform(25).angle = 1.6856040266847538;  % rad
smiData.RigidTransform(25).axis = [-0.32059547542984834 0.32059547542984823 0.89131200051823567];
smiData.RigidTransform(25).ID = "AssemblyGround[Gripper-1:Link-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(26).translation = [0.027638510516534265 -0.0042991418137071028 -0.0032726321026566724];  % m
smiData.RigidTransform(26).angle = 3.139873280503874;  % rad
smiData.RigidTransform(26).axis = [0.70710651988886253 0.70710651988886752 -0.00085968675474654838];
smiData.RigidTransform(26).ID = "AssemblyGround[Arm_2-1:Servo Micro-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(27).translation = [0.029538510516534167 -0.016928803425112663 -0.004085890217849629];  % m
smiData.RigidTransform(27).angle = 0;  % rad
smiData.RigidTransform(27).axis = [0 0 0];
smiData.RigidTransform(27).ID = "AssemblyGround[Arm_2-1:Arm 2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(28).translation = [-0.036211489483466031 -0.0042991010175531582 -0.038106187810289814];  % m
smiData.RigidTransform(28).angle = 1.5707963267948857;  % rad
smiData.RigidTransform(28).axis = [1 0 0];
smiData.RigidTransform(28).ID = "AssemblyGround[Arm_2-1:Servo MG996R-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(29).translation = [-0.0023055352111781731 0.022622458092315355 0.019168167664094793];  % m
smiData.RigidTransform(29).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(29).axis = [-0.70710678118654746 0 0.70710678118654757];
smiData.RigidTransform(29).ID = "AssemblyGround[Arm_3-1:Servo Micro-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(30).translation = [-0.0023055352111782702 0.0078724580923153556 -0.0041318323359054165];  % m
smiData.RigidTransform(30).angle = 0;  % rad
smiData.RigidTransform(30).axis = [0 0 0];
smiData.RigidTransform(30).ID = "AssemblyGround[Arm_3-1:Arm 3-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(31).translation = [0.0065601733872995248 0.019385953644928119 -0.033616875914901909];  % m
smiData.RigidTransform(31).angle = 0;  % rad
smiData.RigidTransform(31).axis = [0 0 0];
smiData.RigidTransform(31).ID = "RootGround[BaseLink-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(13).mass = 0.0;
smiData.Solid(13).CoM = [0.0 0.0 0.0];
smiData.Solid(13).MoI = [0.0 0.0 0.0];
smiData.Solid(13).PoI = [0.0 0.0 0.0];
smiData.Solid(13).color = [0.0 0.0 0.0];
smiData.Solid(13).opacity = 0.0;
smiData.Solid(13).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.089721318791525201;  % kg
smiData.Solid(1).CoM = [-0.000103814223568812 19.422934781166781 -7.0575678960388881];  % mm
smiData.Solid(1).MoI = [66.937092739878324 70.695447549394487 77.272872070026267];  % kg*mm^2
smiData.Solid(1).PoI = [7.9679090392684797 -0.00027714050222831313 6.6609670610196347e-05];  % kg*mm^2
smiData.Solid(1).color = [1 1 1];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "Rotating Waste*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.033876630295594158;  % kg
smiData.Solid(2).CoM = [-0.49032191241283091 20.358074512903947 -1.2918440699258237e-06];  % mm
smiData.Solid(2).MoI = [5.8478860963115435 6.0267252145453636 9.5874430627169769];  % kg*mm^2
smiData.Solid(2).PoI = [6.4718830445865982e-07 1.5672389689302076e-07 0.27277211248685718];  % kg*mm^2
smiData.Solid(2).color = [0.25098039215686274 0.25098039215686274 0.25098039215686274];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "Servo MG996R*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.093991528357273896;  % kg
smiData.Solid(3).CoM = [0.88930748628568013 25.00000015179576 -2.5740826422795617];  % mm
smiData.Solid(3).MoI = [114.11747096047216 124.42725795815227 197.76858898421952];  % kg*mm^2
smiData.Solid(3).PoI = [1.9115339056411537e-07 -1.3528430253361192 1.9770438996363348e-07];  % kg*mm^2
smiData.Solid(3).color = [1 1 1];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "Circular Base*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.1379847018478447;  % kg
smiData.Solid(4).CoM = [0.0036427036336024949 10.308467305813647 -0.00032059335197869905];  % mm
smiData.Solid(4).MoI = [30.835119656497056 431.68332374126783 412.92742194235387];  % kg*mm^2
smiData.Solid(4).PoI = [0.00035028179109484407 -1.1849157106677841e-05 -0.0033681728276682039];  % kg*mm^2
smiData.Solid(4).color = [1 1 1];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "Arm_1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.015576159256236224;  % kg
smiData.Solid(5).CoM = [24.485056579738181 4.7924503379828085 -1.6045282512684211];  % mm
smiData.Solid(5).MoI = [2.5842316653018664 6.2479155961497739 4.8787428545940354];  % kg*mm^2
smiData.Solid(5).PoI = [0.010017148093103258 0.54612472189822581 -0.83085614601083813];  % kg*mm^2
smiData.Solid(5).color = [1 1 1];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "Base*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.00079876241318469095;  % kg
smiData.Solid(6).CoM = [15.500000000000007 2 0];  % mm
smiData.Solid(6).MoI = [0.0036325042236300992 0.10442422565960369 0.10292175453779942];  % kg*mm^2
smiData.Solid(6).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(6).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = "Link*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0.0053974701398930717;  % kg
smiData.Solid(7).CoM = [4.2500225915138357 9.5131476517421962 -31.889976021591902];  % mm
smiData.Solid(7).MoI = [1.7139108532223843 1.5958276983903421 0.18307767030588401];  % kg*mm^2
smiData.Solid(7).PoI = [0.37341188434821881 6.5121955707659532e-06 1.2431071757242153e-05];  % kg*mm^2
smiData.Solid(7).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = "Grip*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(8).mass = 0.0017572412778761883;  % kg
smiData.Solid(8).CoM = [-3.9398950097444714 0 1.808908813594637];  % mm
smiData.Solid(8).MoI = [0.009502192791113688 0.054669651751985113 0.057671077946502156];  % kg*mm^2
smiData.Solid(8).PoI = [0 -0.0056003557135239669 0];  % kg*mm^2
smiData.Solid(8).color = [1 1 1];
smiData.Solid(8).opacity = 1;
smiData.Solid(8).ID = "Microservo horn*:*?? ?????????";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(9).mass = 0.0071285815431194289;  % kg
smiData.Solid(9).CoM = [-0.42843487706856731 12.510643062129502 0];  % mm
smiData.Solid(9).MoI = [0.4581464493976436 0.42765081401936345 0.70781861394704571];  % kg*mm^2
smiData.Solid(9).PoI = [0 0 0.035781849855651131];  % kg*mm^2
smiData.Solid(9).color = [0.12156862745098039 0.25490196078431371 1];
smiData.Solid(9).opacity = 1;
smiData.Solid(9).ID = "Servo Micro*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(10).mass = 0.0039624199269446122;  % kg
smiData.Solid(10).CoM = [-4.5275913705687696 1 -0.60196930727362208];  % mm
smiData.Solid(10).MoI = [0.14167320068202865 0.67085950123994897 0.5529608201195878];  % kg*mm^2
smiData.Solid(10).PoI = [0 -0.013886112733173262 0];  % kg*mm^2
smiData.Solid(10).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(10).opacity = 1;
smiData.Solid(10).ID = "Gear Arm 1*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(11).mass = 0.0028244899707033041;  % kg
smiData.Solid(11).CoM = [-17.370546750743475 1.7437179323548235 0.40231023320767639];  % mm
smiData.Solid(11).MoI = [0.10249998648924344 0.54141333132696878 0.44993490963466681];  % kg*mm^2
smiData.Solid(11).PoI = [0.0021040568313833354 0.058922340817105484 -0.012935612981905613];  % kg*mm^2
smiData.Solid(11).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(11).opacity = 1;
smiData.Solid(11).ID = "Gear Arm 2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(12).mass = 0.080335404973810018;  % kg
smiData.Solid(12).CoM = [-44.301030823737541 12.730203302188647 2.0508592474980816];  % mm
smiData.Solid(12).MoI = [21.25745814669855 104.52010849934938 117.2265982816088];  % kg*mm^2
smiData.Solid(12).PoI = [0.010477131920287354 0.81701872534565867 -0.25423125853984885];  % kg*mm^2
smiData.Solid(12).color = [1 1 1];
smiData.Solid(12).opacity = 1;
smiData.Solid(12).ID = "Arm 2*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(13).mass = 0.020451254914597893;  % kg
smiData.Solid(13).CoM = [0.037358468133722789 -4.1032325236279963 18.449183285021832];  % mm
smiData.Solid(13).MoI = [5.0954754177574983 6.0743432546488227 2.9612438436111672];  % kg*mm^2
smiData.Solid(13).PoI = [0.39169160342408993 0.011228103814175669 -0.0030202973543223162];  % kg*mm^2
smiData.Solid(13).color = [1 1 1];
smiData.Solid(13).opacity = 1;
smiData.Solid(13).ID = "Arm 3*:*Default";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(4).Rz.Pos = 0.0;
smiData.RevoluteJoint(4).ID = "";

smiData.RevoluteJoint(1).Rz.Pos = 179.83977895785137;  % deg
smiData.RevoluteJoint(1).ID = "[RotatingWasteLink-2:-:Arm_1-1]";

smiData.RevoluteJoint(2).Rz.Pos = -89.399648290834108;  % deg
smiData.RevoluteJoint(2).ID = "[Arm_1-1:-:Arm_2-1]";

smiData.RevoluteJoint(3).Rz.Pos = -89.999999999999957;  % deg
smiData.RevoluteJoint(3).ID = "[Arm_3-1:-:Gripper-1]";

smiData.RevoluteJoint(4).Rz.Pos = 89.74525010678957;  % deg
smiData.RevoluteJoint(4).ID = "[BaseLink-1:-:RotatingWasteLink-2]";

