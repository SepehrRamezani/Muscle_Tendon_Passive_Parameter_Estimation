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
%%% Pelvis
modeljointSet=osimmodel.getJointSet();
Pelvisjoint=modeljointSet.get(0);
Pelvisweldjoint=WeldJoint.safeDownCast(Pelvisjoint);
hipfram=Pelvisweldjoint.get_frames(0);
hipfram.set_orientation(Vec3(0,0,(90-Data.(Coordlable).Hipangle)/180*pi()));
%%% Hip_flexion
modelCoordSet = osimmodel.getCoordinateSet();
Hipcoord = modelCoordSet.get(0);
Hipcoord.setDefaultValue(Data.(Coordlable).Hipangle/180*pi());
%%% Knee_corrdiante
Kneecoord = modelCoordSet.get(1);
Kneecoord.setDefaultValue(Kneeangle);
%%% ankle_corrdiante
Anklecoord = modelCoordSet.get(3);
Anklecoord.setDefaultValue(Ankleangle);
%% setup muscle properties

if Data.DeGrooteflage
    DeGrooteFregly2016Muscle().replaceMuscles(osimmodel);
end
c=0;
%% Setup Muscles
for m = 0:osimmodel.getMuscles().getSize()-1
    Muscname=osimmodel.getMuscles().get(m).getName();
    frcset = osimmodel.updForceSet().get(Muscname);
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
state=osimmodel.initSystem();
for corindx = 1:length(Data.ActiveCoordinates)
    %%% Unlock the active coordinate
    osimmodel.updCoordinateSet().get(Data.ActiveCoordinates(corindx)).set_locked(false);
    %%% Adding the coordinate actuator
    addCoordinateActuator(osimmodel, Data.ActiveCoordinates(corindx), optForce)
end
osimmodel.initSystem();
KneeCoor=osimmodel.updCoordinateSet().get('knee_angle_r');

newi=0;

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
            
            KneeCoor.setValue(state, q);
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
function addCoordinateActuator(model, coordName, optForce)

import org.opensim.modeling.*;

coordSet = model.updCoordinateSet();

actu = CoordinateActuator();
actu.setName(append(coordName,'_act'));
actu.setCoordinate(coordSet.get(coordName));
actu.setOptimalForce(optForce);
actu.setMinControl(-1);
actu.setMaxControl(1);
% model.updForceSet().add(actu);
model.addForce(actu);

end
