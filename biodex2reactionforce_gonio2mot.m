clear all;
close all;
clc;
% Some times there is no need to import raw data because all data will
% save in FinalDatafor first time. readflage=1 means import files again.
readflage= 0;
% folder=uigetdir(); % get Data directory
SubjectNumber='T002';
Project='P006';
folder=append('C:\MyCloud\OneDriveUcf\Real\Simulation\Source\',Project,'\',SubjectNumber);
Datafolder=append(folder,'\Data');
results_folder = append(folder,'\Result');
Pardata=importdata(append(Datafolder,"\","Parameters.csv"));
ResultData.info.ForceRatio=Pardata.data(1);
ResultData.info.M_ThresholdMin=Pardata.data(2);
ResultData.info.M_ThresholdMax=Pardata.data(3);
Calibration_Limits=Pardata.data([4,5]);
psname=append(Project,'_',SubjectNumber,'_');
CoordinateName=["RKnee","RAnkle"];
MovementType=["Fl","Dorsi","Plant"];
FixedJoint=["H90","H70","H55","H40","H25","H10","K0","K30","K60","K90","K100"];
SpeedType=["0","2","5","Q"];
% Subjectname =append(psname,"_",CoordinateName,"_");
% Terials1=["Fl"];
% Terials2=["IsoK60"];
ArmWeight=2.72;
ArmCOM=0.218;
Fdata=[];
counter=0
k=0;
% DStime=0.0005192; % desired sampling time
DStime=0.01;
%
filename="";
%% Making File name
%for knee
for T2=1:6
    if T2>3
        counter=counter+1;
        filename(counter)=append(CoordinateName(1),'_',MovementType(1),'_',FixedJoint(T2),"_",SpeedType(4));
    else
        for T3=1:length(SpeedType)
            counter=counter+1;
            filename(counter)=append(CoordinateName(1),'_',MovementType(1),'_',FixedJoint(T2),"_",SpeedType(T3));
            
        end
    end
    
end
% for Aknle
for T1=2:length(MovementType)
    for T2=7:length(FixedJoint)
        counter=counter+1;
        filename(counter)=append(CoordinateName(2),'_',MovementType(T1),'_',FixedJoint(T2));
    end
end
%% Resampling
if (readflage)
    for Count=1:length(filename)
        if Count~=22
            Header=filename(Count);
            Datadr=append(Datafolder,"\",psname,filename(Count),".csv");
            ResampledData=SampleRateCorrection(Datadr,DStime);
            FinalData.(Header).data=ResampledData.data;
            FinalData.(Header).colheaders=ResampledData.colheaders;
        end
    end
    
    save ([Datafolder '\RawData.mat'],'FinalData');
end
%%
load ([Datafolder '\RawData.mat']);
Dataheadermotion=['time\tpelvis_tilt\tpelvis_tx\tpelvis_ty\thip_flexion_r\thip_adduction_r\thip_rotation_r\tknee_angle_r\tsubtalar_angle_r\tankle_angle_r'];
Dataheaderforce=['time\treaction_force_vx\treaction_force_vy\treaction_force_vz\treaction_force_px\treaction_force_py\treaction_force_pz\treaction_torque_x\treaction_torque_y\treaction_torque_z'];
DataheaderEMG=['time\t'];
%getting goniometer calibration coefficient
[P_Gonio_H,P_Gonio_K,P_Gonio_A]= GnCalib(Datafolder,psname,0);
[Weight_Coefficient,P_Biodex_Torque,P_Biodex_Knee_Angle,P_Biodex_Ankle_Angle]= BiodexCalib(Datafolder,psname,Calibration_Limits,0);
for Count=4:length(filename)
    if contains(filename(Count),"Q") & Count~=22
        %         EMGHDdata=[""];
        Header=filename(Count);
        Data=FinalData.(Header).data;
        HData=FinalData.(Header).colheaders;
        [rg,ca]=find(strncmp(HData,'Gn A',2));
        %find Knee Goniometer
        [rk,ck]=find(strncmp(HData,'Gn K',4));
        %find Hip goniometer
        [rh,ch]=find(strncmp(HData,'Gn H',4));
        %find Biodex
        [rb,cb]=find(strncmp(HData,'Biodex',6));
        [r,c]=size(Data);
        %% Process on Motion Data
        
        %knee calibration
        GonK=Data(:,ck(2));
        GonCalibratedK = polyval(P_Gonio_K,GonK);
        %         GonCalibratedK(GonCalibratedK<0)=0;
        %Hip calibration
        GonH=Data(:,ch(2));
        GonCalibratedH = polyval(P_Gonio_H,GonH);
        %Ankle calibration
        GonA=Data(:,ca(2));
        GonCalibratedA = polyval(P_Gonio_A,GonA);
        
        %% Save Motion
        Time=Data(:,1);
        delimiterIn='\t';
        F_fnames = append(psname,char(Header),'_Motion.mot');
        Title='\nversion=1\nnRows=%d\nnColumns=%d\nInDegrees=no\nendheader\n';
        MDatadata = [1,0,0.055,1.059,1,0,0,1,0,0].*ones(r,10);
        MDatadata(:,[1,5,8,10])=[Time,GonCalibratedH,GonCalibratedK,GonCalibratedA];
        Titledata = [r,length(MDatadata(1,:))];
        makefile(Datafolder,F_fnames,Title,Titledata,Dataheadermotion,MDatadata,5,delimiterIn);
        %% Process Force
        %%% Caculating Torque from Arm
        RawAngle=Data(:,cb(2));
        if contains(Header,"Knee")
            BiodexAngle=polyval(P_Biodex_Knee_Angle,RawAngle);
        elseif contains(Header,"Ankle")
            BiodexAngle=polyval(P_Biodex_Ankle_Angle,RawAngle);
        else
            warning('Invalid Data')
        end
        %%% Calcuating torque from biodex arm
        ArmTorque=cos(BiodexAngle)*Weight_Coefficient;
        %%% Calcuating total torque of biodex
        RawTorque=Data(:,cb(1));
        TotalTorque=polyval(P_Biodex_Torque,RawTorque);
        %%% Arm weight compensation
        Mb=TotalTorque-ArmTorque;
        %% Save Force
        F_fnames=append(psname,char(Header),'_Torque.mot');
        FDatadata=[Time,zeros(r,8),Mb];
        Titledata=[r,length(FDatadata(1,:))];
        makefile(Datafolder,F_fnames,Title,Titledata,Dataheaderforce,FDatadata,5,delimiterIn);
        
        %% Finding events
        Event=EventDetection(Header,Time,BiodexAngle);
        Stime=Event(1);
        Etime=Event(2);
        Expindx=find(Data(:,1)>=Stime&Data(:,1)<=Etime);
        plot([BiodexAngle(Expindx)*180/pi(),Mb(Expindx)])
        hold on
        plot(BiodexAngle*180/pi())
        hold off
        figure
        %% Strat reading Simulation files
        ResultData.(Header).('ExpTorque').('full')=[Time(Expindx),Mb(Expindx)];
        ResultData.(Header).('Motion').('full')=[Time(Expindx),GonCalibratedK(Expindx)];
        ResultData.(Header).Events=Expindx;
        
    end
    
end

save (append(results_folder,"\",psname,"ResultData.mat"),'ResultData');

