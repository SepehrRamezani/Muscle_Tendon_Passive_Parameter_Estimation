function Passiveforcegenerate(combinedname,Data,osimmodel)
import org.opensim.modeling.*;
hiplable=append('hip_',Data.whichleg);
kneelable=append('knee_angle_',Data.whichleg);
anklelable=append('ankle_angle_',Data.whichleg);
modeljointSet=osimmodel.getJointSet();

state=osimmodel.initSystem();
muscname=[];
for i=0:1:osimmodel.getMuscles().getSize()-1

    k=0;
    u=0;
    step=30;
    %% finding minimum MTL
    %%Knee deformation
    CurrentMuscle=osimmodel.getMuscles().get(Data.SimMusclename(i+1));
    muscname=[muscname string(CurrentMuscle.getName())];
    Currentcoord=osimmodel.updCoordinateSet().get(Data.ActiveCoordinates);
    rmax=Currentcoord.getRangeMax();
    rmin=Currentcoord.getRangeMin();
    %         Kneecoord.setValue(state, Kneeangle);
    for q=rmin:(rmax-rmin)/step:rmax
        k=k+1;
        Currentcoord.setValue(state, q);
        angle(k,i+1)=Currentcoord.getValue(state);
        osimmodel.realizePosition(state);
        
        degmus=DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle);
        musclelength(k,i+1)=degmus.getLength(state);
        %             osimmodel.assemble(state)
        osimmodel.equilibrateMuscles(state)
        musclemomentarm(k,i+1)=CurrentMuscle.get_GeometryPath().computeMomentArm(state,Currentcoord);
        %             osimmodel.realizeDynamics(state)
        Passivefiberforce(k,i+1)=degmus.getPassiveFiberForce(state);
        Tendonforce(k,i+1)=degmus.getTendonForce(state);
        alpha(k,i+1)=degmus.getPennationAngle(state);
    end
end
differ=Passivefiberforce.*cos(alpha)-Tendonforce;
if differ > 0.01
    fprintf('in %s muscle %s',combinedname,string(CurrentMuscle.getName()));
end
%     plot(rad2deg(angle(:,i+1))',[Passivefiberforce(:,i+1)'])
muscletorque=Tendonforce.*musclemomentarm;
maxpasivetorque=max(abs(muscletorque),[],1);
t = tiledlayout(1,1,'TileSpacing','compact');
ax1 = nexttile;
h=pie(ax1,maxpasivetorque);
ax1.Colormap = hsv(numel(h)/2);
lgd=legend(muscname);
lgd.Layout.Tile = 'east';
title(combinedname)
end