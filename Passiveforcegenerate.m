function Passiveforcegenerate(combinedname,Data,osimmodel)
import org.opensim.modeling.*;
Kneerange=deg2rad(90);
hiplable=append('hip_',Data.whichleg);
kneelable=append('knee_angle_',Data.whichleg);
anklelable=append('ankle_angle_',Data.whichleg);
modeljointSet=osimmodel.getJointSet();

state=osimmodel.initSystem();
for i=0:1:osimmodel.getMuscles().getSize()-1
    k=0;
    u=0;
    step=30;
    %% finding minimum MTL
    %%Knee deformation
    CurrentMuscle=osimmodel.getMuscles().get(Data.SimMusclename(i+1));
    if sum(strcmp(char(CurrentMuscle.getName),Data.ComplianceTendon))&& sum(contains(Data.ActiveCoordinates,'ankle_angle'))
        AnkleCoor=osimmodel.updCoordinateSet().get(Data.ActiveCoordinates);
        rmax=AnkleCoor.getRangeMax();
        rmin=AnkleCoor.getRangeMin();
%         Kneecoord.setValue(state, Kneeangle);
        for q=rmin:(rmax-rmin)/step:rmax
            u=u+1;
            AnkleCoor.setValue(state, q);
            angle(u)=Kneecoord.getValue(state);
            osimmodel.realizePosition(state);
            musclelength(u)=CurrentMuscle.getLength(state);
            Passivefiberforce(u)=CurrentMuscle.getPassiveFiberForce(state);
            Tendonforce(u)=CurrentMuscle.getTendonForce(state);
            alpha(u)=CurrentMuscle.getPennationAngle(state);
        end
    else
        Kneecoord=osimmodel.updCoordinateSet().get(Data.ActiveCoordinates);
        for q=0:Kneerange/step:Kneerange
            k=k+1;
            Kneecoord.setValue(state, q);
            angle(k,i)=Kneecoord.getValue(state);
            osimmodel.realizePosition(state);
            
            degmus=DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle);
            musclelength(k,i)=degmus.getLength(state);
%             osimmodel.assemble(state)
            osimmodel.equilibrateMuscles(state)
%             osimmodel.realizeDynamics(state)
            Passivefiberforce(k,i)=degmus.getPassiveFiberForce(state);
            Tendonforce(k,i)=degmus.getTendonForce(state);
            alpha(k,i)=degmus.getPennationAngle(state);
        end
    end
    differ=Passivefiberforce.*cos(alpha)-Tendonforce;
    plot(rad2deg(angle)',[Passivefiberforce'])
    Muscleinfo.MinMTLength(i+1)=min(musclelength);
    
   
end
osimmodel.initSystem();
