clear all;
close all;
clc;
% Some times there is no need to import raw data because all data will
% save in FinalDatafor first time. readflage=1 means import files again.
readflage=0;
plot_flag=0;
% folder=uigetdir(); % get Data directory

Project='P006';
SubjectNumber=["06","07","08","09","10","11","12","13","14","15"];
% SubjectNumber=["07"];

% SubjectNumber=["06"];

SubjectNumber=append("T0",SubjectNumber);




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
basefolder=fullfile('C:\MyCloud\OneDriveUcf\Real\Simulation\Source',Project);
if readflage
    resultfile=[basefolder '\RawData.mat'];
    if isfile(resultfile)
        load(resultfile);
    end
   for S=1:length(SubjectNumber)
        psname=append(Project,'_',SubjectNumber(S));
        folder=fullfile(basefolder,SubjectNumber(S));
        Datafolder=fullfile(folder,'Data');
        
        Pardata=importdata(append(Datafolder,"\","Parameters.csv"));
        whichleg=string(extractBetween(Pardata.textdata{1},"=",","));
        ResultData.info.whichleg=whichleg;
        ResultData.info.ForceRatio=Pardata.data(1);
        ResultData.info.M_ThresholdMin=Pardata.data(2);
        ResultData.info.M_ThresholdMax=Pardata.data(3);
        ResultData.info.MaxDorsi_Calib=Pardata.data(4);
        ResultData.info.MaxPlant_Calib=Pardata.data(5);
        ResultData.info.optForce=Pardata.data(6);
        
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
                FinalData.(SubjectNumber(S))=ResultData;
            end
        end
    end
    fprintf('Data of %s has converted \n',SubjectNumber(S));
  end
    
    save ([basefolder '\RawData.mat'],'FinalData');
    
end
%%
load ([basefolder '\RawData.mat']);
delimiterIn='\t';
% joints=["hip","walker_knee","patellofemoral","ankle","mtp","subtalar"];
% Data.joints(2:end)=makingstr(Data.joints(2:end),Data.whichleg);
Dataheadermotion=['time' delimiterIn '/jointset/hip_r/hip_flexion_r/value' ...
    delimiterIn '/jointset/walker_knee_r/knee_angle_r/value' ...
    delimiterIn '/jointset/ankle_r/ankle_angle_r/value' ...
    delimiterIn 'BiodexAngle'];
Dataheaderforce=['time' delimiterIn '/forceset/knee_angle_r_act'];


TitleM='\nversion=1\nnRows=%d\nnColumns=%d\ninDegrees=no\nendheader\n';
Title='\ninDegrees=no\nnum_controls=1\nnum_derivatives=0\nDataType=double\nversion=3\nnRows=%d\nnColumns=%d\nendheader\n';

%getting goniometer calibration coefficient
for S=1:length(SubjectNumber)
    
  
 
        psname=append(Project,'_',SubjectNumber(S));
        folder=fullfile(basefolder,SubjectNumber(S));
        results_folder = fullfile(folder,'Result');
        resultfile=append(results_folder,"\",psname,"_ResultData.mat");
        if isfile(resultfile)
            load(resultfile);
        end
        Datafolder=fullfile(folder,'Data');
        [Ph,Pk,Pa,P_Bidoex_Calibration,Torquerefs,P_Bidoex_Motion_Calibration]= GnCalib(Datafolder,psname,FinalData.(SubjectNumber(S)).info,1);
        whichleg = FinalData.(SubjectNumber(S)).info.whichleg;
        optForce = FinalData.(SubjectNumber(S)).info.optForce;
        if contains(whichleg,'l')
            Dataheadermotion=strrep(Dataheadermotion,'_r','_l');
            Dataheaderforce=strrep(Dataheaderforce,'_r','_l');
            Biodexcof=[pi(),1];
        end
        for T1=1:length(Joints)
            if contains(Joints(T1),"Knee")
                Terials2=["H90","H55","H15"];
            else
                Terials2=["K90","K45","K0"];
            end
            %/
            for T2=1:length(Terials2)
                for T3=1:length(Terials3)
                    EMGHDdata=[""];
                    filename=append(psname,'_',upper(whichleg),Joints(T1),'_',Terials2(T2),'_',Terials3(T3));
                    Data=FinalData.(filename).data;
                    HData=FinalData.(filename).colheaders;
                    [rg,ca]=find(contains(HData,'Goni')&contains(HData,'Ankle'));
%                     find Knee Goniometer
                    [rk,ck]=find(contains(HData,'Goni')&contains(HData,'Knee'));
%                     find Hip goniometer
                    [rh,ch]=find(contains(HData,'Goni')&contains(HData,'Hip'));
%                     find Biodex
                    [rb,cb]=find(contains(HData,'Biodex'));
%                     find EMG
                    [re,ce]=find(contains(HData,'EMG')&~contains(HData,'RMS'));
                    [r,c]=size(Data);
                    % Process on Motion Data
%                     knee calibration
                    GonK=Data(:,ck(2));
                    GonCalibratedK = polyval(Pk,GonK);
                    GonCalibratedK=GonCalibratedK-GonCalibratedK(1);
%                     Hip calibration
                    GonH=Data(:,ch(2));
                    GonCalibratedH = polyval(Ph,GonH);
%                     Ankle calibration
                    GonA=Data(:,ca(1));
                    GonCalibratedA = polyval(Pa,GonA);
%                     Biodex angle
                    
                    if contains(Joints(T1),"Knee")
                        BiodexAngle=1*(polyval(P_Bidoex_Motion_Calibration([1,2]),Data(:,cb(2)))*pi()/180);
                        Thrsh=0.04;
                        Dataheaderforce=strrep(Dataheaderforce,'ankle','knee');
                        Events=EventDetection(ResultData.info.DStime,[BiodexAngle,Data(:,cb(1))],Thrsh,1);
                        Torqueref=Torquerefs(1);
                        MTable=[Data(:,1),GonCalibratedH,GonCalibratedK,GonCalibratedA,BiodexAngle];
                    else
                        BiodexAngle=-1*(polyval(P_Bidoex_Motion_Calibration([1,3]),Data(:,cb(2)))*pi()/180);
                        Thrsh=0.07;
                        Dataheaderforce=strrep(Dataheaderforce,'knee','ankle');
                        Events=EventDetection(ResultData.info.DStime,[BiodexAngle,Data(:,cb(1))],Thrsh,1);
                        Torqueref=Torquerefs(2);
                        MTable=[Data(:,1),GonCalibratedH,GonCalibratedK,GonCalibratedA,BiodexAngle];
                    end
                    
                    % Finding events
                    Sindx=Events.EventEtime(1);
                    if Sindx<=0
                        warning('%s is cropped ... ',filename)
                        Sindx=1;
                    end
                    Eindx=Events.EventEtime(end);
                    % Save Motion
                    F_fnames=append(char(filename),'_Motion.mot');
                    TrimMTable=MTable(Sindx:Eindx,:);
                    %% removing time offset
                                TrimMTable(:,1)=TrimMTable(:,1)-TrimMTable(1,1);
                    [TMr,TMc]=size(TrimMTable);
                    Titledata=[TMr TMc];
                    makefile(Datafolder,F_fnames,TitleM,Titledata,Dataheadermotion,TrimMTable,5,delimiterIn);
                    % Process Force
                    %% Caculating Torque from Arm
                    ArmTorque=cos(BiodexAngle)*Torqueref;
                    %% Calcuating Torque from biodex
                    x=1*Data(:,cb(1)); %data of a trial
                    TotalTroque=polyval(P_Bidoex_Calibration,x);
                    NetTorque=TotalTroque-ArmTorque;
                    
                    JointControl=1*(NetTorque/optForce);
                    % Save Force
                    F_fnames=append(char(filename),'_Torque.sto');
                    FData=[TrimMTable(:,1),JointControl(Sindx:Eindx,1)];
                    [TFr,TFc]=size(FData);
                    Titledata=[TFr TFc];
                    makefile(Datafolder,F_fnames,Title,Titledata,Dataheaderforce,FData,7,delimiterIn);
                    % Strat reading Simulation files
                    ResultData.(char(filename)).('ExpTorque').('Trimed')=FData;
                    ResultData.(char(filename)).('ExpMotion').('Trimed')=TrimMTable;
                    if plot_flag
                    t=tiledlayout(5,1);
                    title(t,filename)
                    nexttile
                    plot(Data(:,1),JointControl)
                    hold on
                    plot(FData(:,1),FData(:,2))
                    hold off
                    ylabel('Control')
                    nexttile
                    plot(TrimMTable(:,5)*180/3.14)
                    ylabel('Biodex A')
                    nexttile
                    plot(Data(1:end-1,1),diff(MTable(:,5))./diff(MTable(:,1)));
                    hold on
                    plot(TrimMTable(1:end-1,1),diff(TrimMTable(:,5))./0.2);
                    hold off
                    ylabel('Biodex S')
                    nexttile
                    plot(TrimMTable(:,1),TrimMTable(:,3)*180/3.14)
                    ylabel('Knee')
                    nexttile
                    plot(TrimMTable(:,1),TrimMTable(:,4)*180/3.14);
                    ylabel('Ankle')
                    figure
                    fprintf('%s is done \n',filename);
                    end
                end
            end
         end
        
        mkdir(results_folder)
        save (resultfile,'ResultData');
end
