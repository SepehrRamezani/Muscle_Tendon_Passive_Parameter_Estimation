function [osimmodel,Muscleinfo]=Modelcreator(Coordlable,Data,osimmodel)
import org.opensim.modeling.*;
% osimmodel = Model('subject_walk_armless_RLeg_justknee.osim');
Kneerange=90/180*3.14;
optForce=3000;
%% change the name
osimmodel.setName(Coordlable)
%% Setup angles
Ankleangle=Data.(Coordlable).Ankleangle/180*pi();
Kneeangle=Data.(Coordlable).Kneeangle/180*pi();
modeljointSet=osimmodel.getJointSet();
Currjoint=modeljointSet.get('ground_pelvis');
% Pelvisweldjoint=WeldJoint.safeDownCast(Pelvisjoint);
hipfram=Currjoint.get_frames(0);
hipfram.set_orientation(Vec3(0,0,(90-Data.(Coordlable).Hipangle)/180*pi()));
hipfram.set_translation(Vec3(0,0.9,0));

%% setup muscle properties

if Data.DeGrooteflage
    DeGrooteFregly2016Muscle().replaceMuscles(osimmodel);
end
c=0;
%% Configure Muscles and remove all other acuators
for m = 0:osimmodel.getActuators().getSize()-1
%     frcset.getName()
    frcset = osimmodel.getActuators().get(m);
    if ~sum(strcmp(char(frcset.getName()), Data.SimMusclename))
        isremove=osimmodel.updForceSet().remove(frcset);
    else
        c=c+1;
        musc=Muscle.safeDownCast(frcset);
        
        musc.set_ignore_activation_dynamics(true);
        
        if Data.DeGrooteflage
            %                 if sum(strcmp(char(frcset.getName()), Data.ComplianacMusclename))
            musc.set_ignore_tendon_compliance(false);
            %                 else
            %                     musc.set_ignore_tendon_compliance(true);
            %                 end
            dgf = DeGrooteFregly2016Muscle.safeDownCast(musc);
            dgf.set_min_control(0.0);
            dgf.set_max_control(0.0);
            dgf.set_active_force_width_scale(1);
            dgf.set_tendon_compliance_dynamics_mode('implicit');
            dgf.set_ignore_passive_fiber_force(false);
        else
            musc.set_min_control(0.0);
            musc.set_max_control(0.0);
        end
        
        
        % if strcmp(char(musc.getName()), 'soleus_r')
        %   dgf.set_active_force_width_scale(10);
        %   dgf.set_ignore_passive_fiber_force(true);
        %   musc.set_max_isometric_force(1.5 * musc.get_max_isometric_force());
        %   dgf.set_ignore_passive_fiber_force(true);
        % end
        
    end
    
    
end
fgroupnames=ArrayStr();
osimmodel.getForceSet().getGroupNames(fgroupnames)
for fg = 0:1:fgroupnames.getSize()-1
osimmodel.getForceSet().removeGroup(fgroupnames.get(fg))
end
state=osimmodel.initSystem();
%% Configure markers
for mar = 1:1:osimmodel.getMarkerSet.getSize()
    osimmodel.getMarkerSet().remove(0);
end
%% setting up the constraints
const=osimmodel.getConstraintSet.get('patellofemoral_knee_angle_l_con');
osimmodel.updConstraintSet.remove(const);

%% Configure Joints
jonames=ArrayStr();
osimmodel.getJointSet().getNames(jonames);
for jo = 0:1:jonames.getSize()-1 
    if ~sum(strcmp(char(jonames.get(jo)), Data.joints))
        curjoint=osimmodel.getJointSet.get(jonames.get(jo));
        osimmodel.updJointSet().remove(curjoint);
    end
end



%% Configure Bodies
bonames=ArrayStr();
osimmodel.getBodySet().getNames(bonames);
for bo = 0:1:bonames.getSize()-1   
    if ~sum(strcmp(char(bonames.get(bo)), Data.bodies))
        curbody=osimmodel.getBodySet.get(bonames.get(bo));
        osimmodel.updBodySet().remove(curbody);
    end
end


osimmodel.initSystem();
newi=0;
%% Change the joints type
for i=1:1:length(Data.Weldjoints) 
modeljointSet=osimmodel.getJointSet();
Currjoint=modeljointSet.get(Data.Weldjoints(i));
CurrjointParent=Currjoint.get_frames(0);
CurrjointChild=Currjoint.get_frames(1);
JointWelded=WeldJoint();
JointWelded.setName(Currjoint.getName())
JointWelded.set_frames(0,CurrjointParent)
F1=JointWelded.get_frames(0);
JointWelded.connectSocket_parent_frame(F1);
JointWelded.set_frames(1,CurrjointChild)
F2=JointWelded.get_frames(1);
JointWelded.connectSocket_child_frame(F2)
osimmodel.updJointSet().remove(Currjoint);
osimmodel.addJoint(JointWelded);
osimmodel.initSystem();
end
hipjoint=modeljointSet.get('hip_r');
hipfram=hipjoint.upd_frames(0);
hipfram.set_orientation(Vec3(0,0,(Data.(Coordlable).Hipangle)./180*pi()));

for m = 0:osimmodel.getCoordinateSet().getSize()-1
    Coord=osimmodel.getCoordinateSet().get(m);
    Coord.setDefaultValue(0);
    Coord.set_locked(true)
end
for corindx = 1:length(Data.ActiveCoordinates)
    %%% Unlock the active coordinate
    if strcmp(char(Data.ActiveCoordinates(corindx)),'knee_angle_r')
        osimmodel.updCoordinateSet().get(Data.ActiveCoordinates(corindx)).set_locked(false);
        osimmodel.updCoordinateSet().get('knee_angle_r_beta').set_locked(false);        
    end
    %%% Adding the coordinate actuator
    addCoordinateActuator(osimmodel, Data.ActiveCoordinates(corindx), optForce)
end
%%% Knee_corrdiante
modelCoordSet = osimmodel.getCoordinateSet();
Kneecoord = modelCoordSet.get('knee_angle_r');
Kneecoord.setDefaultValue(Kneeangle);
%%% ankle_corrdiante
Anklecoord = modelCoordSet.get('ankle_angle_r');
Anklecoord.setDefaultValue(Ankleangle);


osimmodel.initSystem();

%% change TSL to avoid buckling and save muscle information
for i=0:1:osimmodel.getMuscles().getSize()-1
    k=0;
    u=0;
    %% finding minimum MTL
    %%Knee deformation
    CurrentMuscle=osimmodel.getMuscles().get(Data.SimMusclename(i+1));
    
    
    if sum(strcmp(char(CurrentMuscle.getName),Data.ComplianacMusclename))&& sum(contains(Data.ActiveCoordinates,'ankle_angle_r'))
        AnkleCoor=osimmodel.updCoordinateSet().get('ankle_angle_r');
        for q=-Ankleangle:Ankleangle/20:Ankleangle
            u=u+1;
            AnkleCoor.setValue(state, q);
            osimmodel.realizePosition(state);
            musclelength(k+u)=CurrentMuscle.getLength(state);
        end
    else
        for q=0:Kneerange/20:Kneerange
            k=k+1;
            
            Kneecoord.setValue(state, q);
            osimmodel.realizePosition(state);
            musclelength(k)=CurrentMuscle.getLength(state);
        end
    end
    
    Muscleinfo.MinMTLength(i+1)=min(musclelength);
    
    if Muscleinfo.MinMTLength(i+1) < CurrentMuscle.get_tendon_slack_length()
        warning('buckeling will be happend in %s',CurrentMuscle.getName())
        %% "0.99" is for making TSL a little bit smaller than minimum MT length   
        TSlack(i+1)=0.99*Muscleinfo.MinMTLength(i+1);
        CurrentMuscle.set_tendon_slack_length(TSlack(i+1));
        Muscleinfo.TSlack(i+1)=TSlack(i+1);
    else
        
        Muscleinfo.TSlack(i+1)=CurrentMuscle.get_tendon_slack_length();
        Muscleinfo.MaxIso(i+1)=CurrentMuscle.get_max_isometric_force();
    end
    %     dgf = DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle);
    Muscleinfo.Passive(i+1)=DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).get_passive_fiber_strain_at_one_norm_force();
    if sum(strcmp(char(CurrentMuscle.getName()), Data.ComplianacMusclename))
        newi=newi+1;
        Muscleinfo.Tstrain(newi)= DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).get_tendon_strain_at_one_norm_force();
    end
end
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
