function [osimmodel]=Robotic_Modelcreator(Coordlable,Data,osimmodel)
import org.opensim.modeling.*;
% osimmodel = Model('subject_walk_armless_RLeg_justknee.osim');
Kneerange=100/180*pi();
hiplable="Hip_Joint";
kneelable="Knee_Joint";
anklelable="Ankle_Joint";

%% change the name
osimmodel.setName(Coordlable)
%% Setup angles
Ankleangle=Data.(Coordlable).Ankleangle;
Kneeangle=Data.(Coordlable).Kneeangle;
Hipangle=Data.(Coordlable).Hipangle;

%% setting up the constraints

osimmodel.initSystem();
newi=0;
%% Change the joints type

modeljointSet=osimmodel.getJointSet();
for i=1:1:length(Data.Weldjoints)
    Currjoint=modeljointSet.get(Data.Weldjoints(i));
        CurrjointParent=Currjoint.get_frames(0);
        CurrjointChild=Currjoint.get_frames(1);
        ModifedJoint=WeldJoint();
        ModifedJoint.setName(Currjoint.getName());
        ModifedJoint.set_frames(0,CurrjointParent);
        F1=ModifedJoint.get_frames(0);
        ModifedJoint.connectSocket_parent_frame(F1);
        ModifedJoint.set_frames(1,CurrjointChild)
        F2=ModifedJoint.get_frames(1);
        ModifedJoint.connectSocket_child_frame(F2);
%         rJoint=osimmodel.updJointSet().get(i-1);
        osimmodel.updJointSet().remove(Currjoint);
        osimmodel.addJoint(ModifedJoint);
        osimmodel.initSystem();
end
grcoor=osimmodel.getJointSet().get('ground_pelvis').clone;
osimmodel.updJointSet().remove(osimmodel.getJointSet().get('ground_pelvis'));
osimmodel.getJointSet().insert(0,grcoor)
%%% lock the unsed joints and add actuator
% for m = 0:osimmodel.getCoordinateSet().getSize()-1
%     Coord=osimmodel.getCoordinateSet().get(m);
%     
%     if any(contains(Data.ActiveCoordinates,char(Coord.getName)))
%     Coord.set_locked(false);
%     else
%     
%     Coord.setDefaultValue(0);
%     Coord.set_locked(true)
%     end
% end

osimmodel.initSystem();
%%% Setup Hip coordinate
modelCoordSet = osimmodel.getCoordinateSet();
if any(contains(Data.Weldjoints,hiplable))
    hipjoint=modeljointSet.get(hiplable);
    pelvisparentframe=hipjoint.upd_frames(0);
    pelvisparentframe.set_orientation(Vec3(0,0,Hipangle));
else
    hipcoord = modelCoordSet.get("Hip");
    hipcoord.setDefaultValue(Hipangle);
end
for m = 0:osimmodel.getCoordinateSet().getSize()-1
    Coord=osimmodel.getCoordinateSet().get(m);
    Coord.set_locked(false);
    addCoordinateActuator(osimmodel,char(Coord.getName), 1)
end
osimmodel.initSystem();
%%% Setup Knee_cordiante
Kneecoord = modelCoordSet.get("Knee");
Kneecoord.setDefaultValue(Kneeangle);
Kneecoord.setRangeMax(deg2rad(90));
Kneecoord.setRangeMin(deg2rad(-5));
osimmodel.initSystem();


osimmodel.print(Data.(Coordlable).ModelPath);

end
function addCoordinateActuator(osimmodel, coordName, optForce)

import org.opensim.modeling.*;
coordSet = osimmodel.updCoordinateSet();
actu = CoordinateActuator();
actu.setName(append(coordName,'_act'));
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optForce);
actu.setMinControl(-1);
actu.setMaxControl(1);
% model.updForceSet().add(actu);
osimmodel.addForce(actu);
end
