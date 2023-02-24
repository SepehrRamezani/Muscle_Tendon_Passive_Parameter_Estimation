clear all
% close all
import org.opensim.modeling.*;
Trc_path=['C:\MyCloud\OneDriveUcf\Real\Simulation\Passive_Parameter_prediction\Robotic_Leg\Data'];
listing = dir(Trc_path);
plotfalg=1;
IKflag=0;
Mnames=[];
Tnames=[];
for i=1:length(listing)
    curname=string(listing(i,1).name);
    if contains(curname,".trc")&contains(curname,"Knee")
        Mnames=[Mnames; curname];
    elseif contains(curname,"Torque")&contains(curname,".mot")&contains(curname,"Knee")
        Tnames=[Tnames; curname];
    else
    end
end

modeldir=append(Trc_path,'/../Model/FinalModel.osim');
model=Model(modeldir);
for S=1:length(Mnames)
    Trialnames= erase(Mnames(S),'_Marker.trc');
    markerdir=fullfile(Trc_path,Mnames(S));
    motion = MarkerData(markerdir);
    outdir=fullfile(Trc_path,append(Trialnames,"_IK.mot"));
    if IKflag
    ikTool=InverseKinematicsTool(fullfile(Trc_path,'IK_Setup.xml'));
    ikTool.setName(Trialnames)
    ikTool.setModel(model);
    ikTool.setResultsDir(Trc_path)
    %             ikTool.set_report_marker_locations(true);
    ikTool.setMarkerDataFileName(markerdir);
    ikTool.setStartTime(motion.getStartFrameTime());
    ikTool.setEndTime(motion.getLastFrameTime());

    ikTool.setOutputMotionFileName(outdir);
    ikTool.run();

    %     ik_marker_Error_file = strrep(outdir, 'IK.mot', 'ik_marker_errors.sto');
    ik_marker_Error_file = strrep(outdir, 'IK.mot', 'ik_marker_errors.sto');
    ik_marker_Error=importdata(ik_marker_Error_file);
    marker_error_RMS=mean(ik_marker_Error.data(:,3));
    [Max_error_RMS,Indx]=max(ik_marker_Error.data(:,3));
    if marker_error_RMS>0.001
        fprintf('Avg and Max marker_RMS of %s is %3.4f and %3.4f at %3.4f\n',Mnames(S),marker_error_RMS,Max_error_RMS,ik_marker_Error.data(Indx,1));
        BadIKData.name=Mnames;
        BadIKData.data=marker_error_RMS;
    end

    end
    ik_result= TableProcessor(outdir).process; 
    torque_result=TableProcessor(fullfile(Trc_path,Tnames(S))).process;
%     torque_result=importdata(fullfile(Trc_path,Tnames(S)));
    kneeangle=mean(ik_result.getDependentColumnAtIndex(7).getAsMat());
    Hipangle=mean(ik_result.getDependentColumnAtIndex(6).getAsMat());
    torque=mean(torque_result.getDependentColumnAtIndex(0).getAsMat());
    FinalData(S,:)=[kneeangle,Hipangle,torque];

end

indx35=find(FinalData(:,2)<40& FinalData(:,2)>30);
Data35=sortrows(FinalData(indx35,:),1);
indx45=find(FinalData(:,2)<50& FinalData(:,2)>40);
Data45=sortrows(FinalData(indx45,:),1);
indx70=find(FinalData(:,2)<75& FinalData(:,2)>65);
Data70=sortrows(FinalData(indx70,:),1);
indx90=find(FinalData(:,2)<95& FinalData(:,2)>85);
Data90=sortrows(FinalData(indx90,:),1);
if plotfalg
plot(Data35(:,1),Data35(:,3))
hold on
plot(Data45(:,1),Data45(:,3))
plot(Data70(:,1),Data70(:,3))
plot(Data90(:,1),Data90(:,3))
legend(string(floor([Data35(1,2),Data45(1,2),Data70(1,2),Data90(1,2)])))
hold off
end

time90=Data90(:,1)*20/90;
newtim90=0:20/(4*length(time90)):20;
newData90=interp1(time90,Data90,newtim90,"linear","extrap");
% % c3dtotrc();