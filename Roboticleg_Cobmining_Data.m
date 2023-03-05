clear all
% close all
import org.opensim.modeling.*;
Trc_path=['C:\MyCloud\OneDriveUcf\Real\Simulation\Passive_Parameter_prediction\Robotic_Leg\Data\Second'];
listing = dir(Trc_path);
plotfalg=1;
IKflag=0;
Torqueflage=0;
Mnames=[];
Tnames=[];
torque=[];
for i=1:length(listing)
    curname=string(listing(i,1).name);
    if contains(curname,".trc")& ~contains(curname,"Knee")
        Mnames=[Mnames; curname];
    elseif contains(curname,"Torque")& ~contains(curname,"Knee")
        Tnames=[Tnames; curname];
    else
    end
end

modeldir=fullfile(Trc_path,'..','..','Model','FinalModel.osim');
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
    if Torqueflage
    torque_result=TableProcessor(fullfile(Trc_path,Tnames(S))).process;
    torque=mean(torque_result.getDependentColumnAtIndex(0).getAsMat());
    end
%     torque_result=importdata(fullfile(Trc_path,Tnames(S)));
    kneeangle=mean(ik_result.getDependentColumnAtIndex(7).getAsMat());

    Hipangle=mean(ik_result.getDependentColumnAtIndex(6).getAsMat());
    FinalData(S,:)=[kneeangle,Hipangle,torque];

end
% Voltage=[1.842 1.725 1.638 1.554 1.458 1.256 1.135 0.857 0.62 0.062 -0.326 -1.67];
load(fullfile(Trc_path,'Voltage.mat'));
% Voltage=FinalData(:,3)-(0.168)+0.018;
Finaltorque=4.61*Voltage - 0.8627;
FinalData(:,3)=Finaltorque;


indx90=find(FinalData(:,2)<95& FinalData(:,2)>85);
Data90=sortrows(FinalData(indx90,:),1,'descend');

% indx110=find(FinalData(:,2)<115& FinalData(:,2)>105);
% Data110=sortrows(FinalData(indx110,:),1);
% indx135=find(FinalData(:,2)<140& FinalData(:,2)>130);
% Data135=sortrows(FinalData(indx135,:),1);
indx35=find(FinalData(:,2)<150& FinalData(:,2)>140);
Data35=sortrows(FinalData(indx35,:),1);
ExpData.Data90=Data90;
ExpData.Data35=Data35;
if plotfalg
    
    plot(90-Data90(:,1),Data90(:,3),"*")
    hold on
%     plot(Data110(:,1),Data110(:,3))
%     plot(90-Data35(:,1),Data35(:,3))

legend(string(floor(180-[Data90(1,2),Data35(1,2)])))
% hold off
end
xlabel('Knee(deg)')
ylabel('Torque(N.m)')
m=(Data90(end,1)-Data90(1,1))/(20);
time90=(Data90(:,1)-Data90(1,1))/m;
ExpData.Data90time=time90;
newtim90=0:20/(5*length(time90)):20;
newData90=interp1(time90,Data90,newtim90,"linear","extrap");
plot(90-newData90(:,1),newData90(:,3))
newData90(:,3)=newData90(:,3)/100;
newData90=[newtim90' newData90];
[TFr,TFc]=size(newData90);
Titledata=[TFr,TFc];
delimiterIn='\t';
Title='\ninDegrees=yes\nnum_controls=1\nnum_derivatives=0\nDataType=double\nversion=3\nnRows=%d\nnColumns=%d\nendheader\n';
F_fnames='Hip90.sto';
Dataheaderforce=append('time',delimiterIn,'knee',delimiterIn,'hip',delimiterIn,'act');
makefile(Trc_path,F_fnames,Title,Titledata,Dataheaderforce,newData90,7,delimiterIn);
save(fullfile(Trc_path,"ExpData.mat"),'ExpData');
% % c3dtotrc();