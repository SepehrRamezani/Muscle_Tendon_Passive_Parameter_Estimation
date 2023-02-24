function [osimmodel]=Robotic_Modelcreator(Coordlable,Data,osimmodel)
import org.opensim.modeling.*;
% osimmodel = Model('subject_walk_armless_RLeg_justknee.osim');
Kneerange=100/180*pi();
hiplable="Hip_Joint";
kneelable="Knee_Joint";
anklelable="Ankle_Joint";
step=max(Data.TorqueSolverinterval,Data.ParamSolverinterval);
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
% grcor=osimmodel.getJointSet().get('ground_pelvis');
% grcoor=grcor.clone;
% osimmodel.updJointSet().remove(grcor);
% osimmodel.getJointSet().insert(0,grcoor)
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

osimmodel.initSystem();
%%% Setup Knee_cordiante
if any(contains(Data.Weldjoints,kneelable))
    kneejoint=modeljointSet.get(kneelable);
    kneeparentframe=kneejoint.upd_frames(0);
    kneeparentframe.set_orientation(Vec3(0,0,Kneeangle));
else
    Kneecoord = modelCoordSet.get("Knee");
    Kneecoord.setDefaultValue(Kneeangle);
    Kneecoord.setRangeMax(deg2rad(90));
    Kneecoord.setRangeMin(deg2rad(-5));
end


%% finding lowest spring length
%%% finding all possible coordinates value
activL=length(Data.ActiveCoordinates);
for c=1:activL
    Currentcoord=modelCoordSet.get(Data.ActiveCoordinates(c));
    rmax=Currentcoord.getRangeMax();
    rmin=Currentcoord.getRangeMin();
    cordvalues=(linspace(rmin,rmax,step));
    elements{c}=cordvalues; %cell array with N vectors to combine
end
combinations = cell(1, numel(elements));  %set up the varargout result
[combinations{:}] = ndgrid(elements{:});
combinations = cellfun(@(x) x(:), combinations,'uniformoutput',false); %there may be a better way to do this
Coboval = [combinations{:}]; % NumberOfCombinations by N matrix. Each row is unique.
%%% feed into model to calculate the Actuator length
state=osimmodel.initSystem();
for i=0:1:osimmodel.getForceSet().getSize()-1
    CurrenSpring=osimmodel.getForceSet().get(i);
    %%% activate the active Force
    if any(contains(Data.ActiveAct,string(CurrenSpring.getName)))
        CurrenSpring.set_appliesForce(true) 
        for cv=1:length(Coboval)
            for c=1:activL
                Currentcoord=modelCoordSet.get(Data.ActiveCoordinates(c));
                Currentcoord.setValue(state, Coboval(cv));
            end
            osimmodel.realizePosition(state);
            pathsp=PathSpring.safeDownCast(CurrenSpring);
            musclelength(cv)=pathsp.getLength(state);
        end

        restpos=min(musclelength);
        pathsp.set_resting_length(restpos);
        pathsp.set_stiffness(Data.stiffness(i+1));
        
    else
    CurrenSpring.set_appliesForce(false) 
    end
end
%% unlock the active coordinates
for m = 0:osimmodel.getCoordinateSet().getSize()-1
    Coord=osimmodel.getCoordinateSet().get(m);
    Coord.set_locked(false);
    addCoordinateActuator(osimmodel,char(Coord.getName), 100)
end



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
