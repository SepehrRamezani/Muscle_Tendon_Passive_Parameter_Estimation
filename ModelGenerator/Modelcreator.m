function [osimmodel,Muscleinfo]=Modelcreator(Coordlable,Data,osimmodel)
import org.opensim.modeling.*;
% osimmodel = Model('subject_walk_armless_RLeg_justknee.osim');
Kneerange=100/180*pi();
hiplable=append('hip_',Data.whichleg);
kneelable=append('knee_angle_',Data.whichleg);
anklelable=append('ankle_angle_',Data.whichleg);

%% change the name
osimmodel.setName(Coordlable)
%% Setup angles
Ankleangle=Data.(Coordlable).Ankleangle;
Kneeangle=Data.(Coordlable).Kneeangle;
Hipangle=Data.(Coordlable).Hipangle;
modeljointSet=osimmodel.getJointSet();


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
osimmodel.getForceSet().getGroupNames(fgroupnames);
for fg = 0:1:fgroupnames.getSize()-1
osimmodel.getForceSet().removeGroup(fgroupnames.get(fg));
end
state=osimmodel.initSystem();
%% Configure markers
for mar = 1:1:osimmodel.getMarkerSet.getSize()
    osimmodel.getMarkerSet().remove(0);
end
%% setting up the constraints
if Data.whichleg==["r"]
    const=osimmodel.getConstraintSet.get('patellofemoral_knee_angle_l_con');
else
    const=osimmodel.getConstraintSet.get('patellofemoral_knee_angle_r_con');
end
osimmodel.updConstraintSet.remove(const);


%% Configure Bodies
bonames=ArrayStr();
osimmodel.getBodySet().getNames(bonames);
for bo = 0:1:bonames.getSize()-1   
    if ~sum(strcmp(char(bonames.get(bo)), Data.bodies))
        curbody=osimmodel.getBodySet.get(bonames.get(bo));
        osimmodel.updBodySet().remove(curbody);
    end
end
%% Configure Joints
jonames=ArrayStr();
osimmodel.getJointSet().getNames(jonames);
for jo = 0:1:jonames.getSize()-1 
    if ~sum(strcmp(char(jonames.get(jo)), Data.joints))
        curjoint=osimmodel.getJointSet.get(jonames.get(jo));
        osimmodel.updJointSet().remove(curjoint);
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
JointWelded.connectSocket_child_frame(F2);
osimmodel.updJointSet().remove(Currjoint);
osimmodel.addJoint(JointWelded);
osimmodel.initSystem();
end
%%% lock the unsed joints and add actuator
for m = 0:osimmodel.getCoordinateSet().getSize()-1
    Coord=osimmodel.getCoordinateSet().get(m);
    
    if sum(contains(Data.ActiveCoordinates,char(Coord.getName)))
    Coord.set_locked(false);
    addCoordinateActuator(osimmodel,char(Coord.getName), Data.optForce)
    else
    
    Coord.setDefaultValue(0);
    Coord.set_locked(true)
    end
end
osimmodel.updCoordinateSet().get(append(kneelable,'_beta')).set_locked(false); 
osimmodel.initSystem();
%%% Setup Hip coordinate
modelCoordSet = osimmodel.getCoordinateSet();
if sum(contains(append('hip_',Data.whichleg), Data.Weldjoints))
    hipjoint=modeljointSet.get(hiplable);
    pelvisparentframe=hipjoint.upd_frames(0);
    pelvisparentframe.set_orientation(Vec3(0,0,Hipangle));
else
    hipcoord = modelCoordSet.get(append('hip_flexion_',Data.whichleg));
    hipcoord.setDefaultValue(Hipangle);
end
%%% Setup Knee_cordiante
Kneecoord = modelCoordSet.get(kneelable);
Kneecoord.setDefaultValue(Kneeangle);
%%% Setup ankle_cordiante
Anklecoord = modelCoordSet.get(anklelable);
Anklecoord.setDefaultValue(Ankleangle);
Anklecoord.setRangeMax(35/180*pi())
%%% Setup Pelvis-ground_cordiante
Pelviscoord=modeljointSet.get('ground_pelvis');
pelvisparentframe=Pelviscoord.get_frames(0);
pelvisparentframe.set_translation(Vec3(0,0.9,0));
% Pelvisweldjoint=WeldJoint.safeDownCast(Pelvisjoint);
if contains(Data.ActiveCoordinates,'ankle')
    pelvisparentframe.set_orientation(Vec3(0,0,Kneeangle-(Hipangle-pi()/2)));
else
    pelvisparentframe.set_orientation(Vec3(0,0,pi()/2-Hipangle));
end
osimmodel.initSystem();

%% change TSL to avoid buckling and save muscle information
for i=0:1:osimmodel.getMuscles().getSize()-1
    k=0;
    u=0;
    %% finding minimum MTL
    %%Knee deformation
    CurrentMuscle=osimmodel.getMuscles().get(Data.SimMusclename(i+1));
    if sum(strcmp(char(CurrentMuscle.getName),Data.ComplianacMusclename))&& sum(contains(Data.ActiveCoordinates,'ankle_angle'))
        AnkleCoor=osimmodel.updCoordinateSet().get(Data.ActiveCoordinates);
        rmax=AnkleCoor.getRangeMax();
        rmin=AnkleCoor.getRangeMin();
        Kneecoord.setValue(state, Kneeangle);
        for q=rmin:(rmax-rmin)/20:rmax
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
        
    end
    Muscleinfo.MaxIso(i+1)=CurrentMuscle.get_max_isometric_force();
    %     dgf = DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle);
    Muscleinfo.Passive(i+1)=DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).get_passive_fiber_strain_at_one_norm_force();
    if sum(strcmp(char(CurrentMuscle.getName()), Data.ComplianacMusclename))
        newi=newi+1;
        Muscleinfo.Tstrain(newi)= DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle).get_tendon_strain_at_one_norm_force();
    end
end
osimmodel.initSystem();

osimmodel.print(Data.(Coordlable).Combinedname);

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
