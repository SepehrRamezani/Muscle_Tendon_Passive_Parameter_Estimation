clear all;
close all;
clc;
% Some times there is no need to import raw data because all data will
% save in FinalDatafor first time. readflage=1 means import files again.
readflage=0;
% folder=uigetdir(); % get Data directory

Project='P006';
SubjectNumber='T008';
folder=append('C:\MyCloud\OneDriveUcf\Real\Simulation\Source\',Project,'\',SubjectNumber);
Datafolder=append(folder,'\Data');
results_folder = append(folder,'\Result');
Pardata=importdata(append(Datafolder,"\","Parameters.csv"));
whichleg=string(extractBetween(Pardata.textdata{1},"=",","));
ResultData.info.ForceRatio=Pardata.data(1);
ResultData.info.M_ThresholdMin=Pardata.data(2);
ResultData.info.M_ThresholdMax=Pardata.data(3);
ResultData.info.MaxDorsi_Calib=Pardata.data(4);
ResultData.info.MaxPlant_Calib=Pardata.data(5);
optForce=Pardata.data(6);
psname=append(Project,'_',SubjectNumber);


Biodexcof=[0,1];

% Terials1=["Fl"];
% Terials2=["H90","H45","H0"];
Joints=["Knee","Ankle"];
Terials3=["L1","L2","L3"];
% Terials1=["Fl"];
% Terials2=["IsoK60"];
ArmWeight=2.72;
ArmCOM=0.27;
Fdata=[];
k=0;
% DStime=0.0005192; % desired sampling time
ResultData.info.DStime=0.2;
%

if readflage
    for T1=1:length(Joints)
        if contains(Joints(T1),"Knee")
            Terials2=["H90","H55","H15"];
        else
            Terials2=["K90","K45","K0"];
        end
        for T2=1:length(Terials2)
            for T3=1:length(Terials3)
                
                filename=append(psname,'_',upper(whichleg),Joints(T1),'_',Terials2(T2),'_',Terials3(T3));
                Datadr=append(Datafolder,"\",filename,'.csv');
                data=importdata(Datadr);
                [Gdata,Gheader]= ReshapingData(data,ResultData.info.DStime);
                FinalData.(filename).data=Gdata;
                FinalData.(filename).colheaders=Gheader;
            end
        end
    end
    
    save ([Datafolder '\RawData.mat'],'FinalData');
    
end
%%
load ([Datafolder '\RawData.mat']);
delimiterIn='\t';
% joints=["hip","walker_knee","patellofemoral","ankle","mtp","subtalar"];
% Data.joints(2:end)=makingstr(Data.joints(2:end),Data.whichleg);
Dataheadermotion=['time' delimiterIn '/jointset/hip_r/hip_flexion_r/value' ...
    delimiterIn '/jointset/walker_knee_r/knee_angle_r/value' ...
    delimiterIn '/jointset/ankle_r/ankle_angle_r/value' ...
    delimiterIn 'BiodexAngle'];
Dataheaderforce=['time' delimiterIn '/forceset/knee_angle_r_act'];
if contains(whichleg,'l')
    Dataheadermotion=strrep(Dataheadermotion,'_r','_l');
    Dataheaderforce=strrep(Dataheaderforce,'_r','_l');
    Biodexcof=[pi(),-1];
end

TitleM='\nversion=1\nnRows=%d\nnColumns=%d\ninDegrees=no\nendheader\n';
Title='\ninDegrees=no\nnum_controls=1\nnum_derivatives=0\nDataType=double\nversion=3\nnRows=%d\nnColumns=%d\nendheader\n';

%getting goniometer calibration coefficient

[Ph,Pk,Pa,P_Bidoex_Calibration,Torquerefs,P_Bidoex_Motion_Calibration]= GnCalib(Datafolder,psname,ResultData.info,0);

for T1=1:length(Joints)
    if contains(Joints(T1),"Knee")
        Terials2=["H90","H55","H15"];
    else
        Terials2=["K90","K45","K0"];
    end
    for T2=1:length(Terials2)
        for T3=1:length(Terials3)
            EMGHDdata=[""];
            filename=append(psname,'_',upper(whichleg),Joints(T1),'_',Terials2(T2),'_',Terials3(T3));
            Data=FinalData.(filename).data;
            HData=FinalData.(filename).colheaders;
            [rg,ca]=find(contains(HData,'Goni')&contains(HData,'Ankle'));
            %find Knee Goniometer
            [rk,ck]=find(contains(HData,'Goni')&contains(HData,'Knee'));
            %find Hip goniometer
            [rh,ch]=find(contains(HData,'Goni')&contains(HData,'Hip'));
            %find Biodex
            [rb,cb]=find(contains(HData,'Biodex'));
            %find EMG
            [re,ce]=find(contains(HData,'EMG')&~contains(HData,'RMS'));
            [r,c]=size(Data);
            %% Process on Motion Data
            %knee calibration
            GonK=Data(:,ck(2));
            GonCalibratedK = polyval(Pk,GonK);
            GonCalibratedK=GonCalibratedK-GonCalibratedK(1);
            %Hip calibration
            GonH=Data(:,ch(2));
            GonCalibratedH = polyval(Ph,GonH);
            %Ankle calibration
            GonA=Data(:,ca(1));
            GonCalibratedA = polyval(Pa,GonA);
            %Biodex angle
            
            %         BiodexAngle=Biodexcof(1)+Biodexcof(2)*(-35.5*Data(:,cb(2))+103)*pi()/180;
            if contains(Joints(T1),"Knee")
                BiodexAngle=polyval(P_Bidoex_Motion_Calibration([1,2]),Data(:,cb(2)))*pi()/180;
                Thrsh=0.04;
                Dataheaderforce=strrep(Dataheaderforce,'ankle','knee');
                Events=EventDetection(ResultData.info.DStime,[BiodexAngle,Data(:,cb(1))],Thrsh,1);
                Torqueref=Torquerefs(1);
                MTable=[Data(:,1),GonCalibratedH,GonCalibratedK,GonCalibratedA,BiodexAngle];
            else
                BiodexAngle=polyval(P_Bidoex_Motion_Calibration([1,3]),Data(:,cb(2)))*pi()/180;
                Thrsh=0.07;
                Dataheaderforce=strrep(Dataheaderforce,'knee','ankle');
                Events=EventDetection(ResultData.info.DStime,[BiodexAngle,Data(:,cb(1))],Thrsh,1);
                Torqueref=Torquerefs(2);
                MTable=[Data(:,1),GonCalibratedH,GonCalibratedK,GonCalibratedA,BiodexAngle];
            end
            
            %% Finding events
            Sindx=Events.EventEtime(1);
            if Sindx<=0
                warning('%s is cropped ... ',filename)
                Sindx=1;
            end
            Eindx=Events.EventEtime(end);
            %% Save Motion
            F_fnames=append(char(filename),'_Motion.mot');
            TrimMTable=MTable(Sindx:Eindx,:);
            %%% removing time offset
%             TrimMTable(:,1)=TrimMTable(:,1)-TrimMTable(1,1);
            [TMr,TMc]=size(TrimMTable);
            Titledata=[TMr TMc];
%              makefile(Datafolder,F_fnames,TitleM,Titledata,Dataheadermotion,TrimMTable,5,delimiterIn);
            %% Process Force
            %%% Caculating Torque from Arm
            % ArmTorque=cos(BiodexAngle*pi()/180)*ArmWeight*9.8*ArmCOM;
            ArmTorque=cos(BiodexAngle)*Torqueref;
            %%% Calcuating Torque from biodex
            x=1*Data(:,cb(1)); %data of a trial
            TotalTroque=polyval(P_Bidoex_Calibration,x);
            NetTorque=TotalTroque-ArmTorque;
            
            JointControl=Biodexcof(2)*(NetTorque/optForce);
            %% Save Force
            F_fnames=append(char(filename),'_Torque.mot');
            FData=[TrimMTable(:,1),JointControl(Sindx:Eindx,1)];
            [TFr,TFc]=size(FData);
            Titledata=[TFr TFc];
%              makefile(Datafolder,F_fnames,Title,Titledata,Dataheaderforce,FData,7,delimiterIn);
            %% Strat reading Simulation files
            ResultData.(char(filename)).('ExpTorque').('full')=FData;
            ResultData.(char(filename)).('ExpMotion').('full')=TrimMTable;
            
            
            
            
            %             CombinedFdata(:,T3)=MData(:,2);
            t=tiledlayout(5,1);
            title(t,filename)
            nexttile
            plot(Data(:,1),JointControl)
            hold on
            plot(FData(:,1),FData(:,2))
            hold off
            nexttile
            plot(TrimMTable(:,5)*180/3.14)
            nexttile
            plot(Data(1:end-1,1),diff(MTable(:,5))./diff(MTable(:,1)));
            hold on
            plot(TrimMTable(1:end-1,1),diff(TrimMTable(:,5))./0.2);
            hold off
            nexttile
            plot(TrimMTable(:,1),TrimMTable(:,3)*180/3.14)
            nexttile
            plot(TrimMTable(:,1),TrimMTable(:,4)*180/3.14);
            figure
            
        end
%             CombinedFdata=mean(CombinedFdata,2); 
%             filenameComb=append(psname,'_',upper(whichleg),Joints(T1),'_',Terials2(T2));
            
%             ResultData.(char(filenameComb)).('ExpTorque').('full')=FData;
%             ResultData.(char(filenameComb)).('ExpMotion').('full')=TrimMTable;
    end
end
mkdir(results_folder)
save (append(results_folder,"\",psname,"_ResultData.mat"),'ResultData');

