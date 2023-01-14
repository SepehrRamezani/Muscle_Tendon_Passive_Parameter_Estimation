clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)
Project='P006';
SubjectNumber=["06","07","08","09","10","11","12","13","14","15"];
% SubjectNumber=["08","09","10","11","12","13","14","15"];

SubjectNumber=append("T0",SubjectNumber);
Data.whichleg="l";
Data.joints=["ground_pelvis","hip","walker_knee","patellofemoral","ankle","mtp","subtalar"];
Data.joints(2:end)=addingleg(Data.joints(2:end),Data.whichleg);
% Data.Weldjoints=["ground_pelvis","hip","mtp","subtalar"];
Data.Weldjoints=["ground_pelvis","mtp","subtalar"];
Data.Weldjoints(2:end)=addingleg(Data.Weldjoints(2:end),Data.whichleg);
Data.bodies=["pelvis","femur","tibia","patella","talus","calcn","toes"];
Data.bodies(2:end)=addingleg(Data.bodies(2:end),Data.whichleg);


Data.DeGrooteflage=1;
% Hipangle=0;%deg
% Basepath=[cd];

% Data.RefStatepath=append(Basepath,'\TorqueSimulation\referenceCoordinates.sto');

Data.TorqueSolverinterval=40;
Data.ParamSolverinterval=50;
% Data.Etime=20;
Data.Stime=0;
Data.PassiveFiberBound=[0.05,0.9];
Data.TendonStrainBound=[0.01,0.1];


Joints=["Ankle","Knee"];
Terials3=["L1","L2","L3"];

%% running just Parameter optimization
Data.justparameterflag=1;
qe=1;
for S=1:length(SubjectNumber)
    Basepath=append(['C:\MyCloud\OneDriveUcf\Real\Simulation\Source'],'\',Project,'\',SubjectNumber(S));
    Pardata=importdata(append(Basepath,"\Data\Parameters.csv"));
    psname=append(Project,'_',SubjectNumber(S));
    Data.optForce=Pardata.data(6);
    Data.RefModelpath=append(Basepath,'\Model\',psname,'_Rajagopal_Scaled.osim');
    
    for T1=1:length(Joints)
        if contains(Joints(T1),"Knee")
            Terials2=["H90","H55","H15"];
            Terials2unm=[90,55,10]*3.14/180;
            %             Terials2=["H90"];
            Data.ActiveCoordinates=["knee_angle"];
            Data.ActiveCoordinates=addingleg(Data.ActiveCoordinates,Data.whichleg);
            Data.Rigidtendon=["bflh","bfsh","recfem","semimem","semiten","vasint","vaslat","vasmed"];
            Data.Rigidtendon=addingleg(Data.Rigidtendon,Data.whichleg);
            Data.ComplianacMusclename=[];
        else
            Terials2=["K90","K45","K0"];
            Terials2unm=[90,45,0]*3.14/180;
            %             Terials2=["K90"];
            Data.ActiveCoordinates=["ankle_angle"];
            Data.ActiveCoordinates=addingleg(Data.ActiveCoordinates,Data.whichleg);
            Data.Rigidtendon=["gaslat","gasmed"];
            Data.Rigidtendon=addingleg(Data.Rigidtendon,Data.whichleg);
            Data.ComplianacMusclename=["gaslat","gasmed"];
            Data.ComplianacMusclename=addingleg(Data.ComplianacMusclename,Data.whichleg);
        end
        % Make sure if you have common name in the Rigidtendon and Compliance
        % Muscle put their name at the end of rigid tendon.
        if ~isempty(Data.ComplianacMusclename)
            Data.SimMusclename=[Data.Rigidtendon(~contains(Data.Rigidtendon,Data.ComplianacMusclename)),Data.ComplianacMusclename];
        else
            Data.SimMusclename=Data.Rigidtendon;
        end
        for T2=1:length(Terials2)
            
            combinedname=append(psname,'_',upper(Data.whichleg),Joints(T1),'_',Terials2(T2));
            Data.(combinedname).Combinedname=append(Basepath,'\Model\',combinedname,'.osim');
            if contains(Joints(T1),"Knee")
                Data.(combinedname).Hipangle=Terials2unm(T2);
                Data.(combinedname).Kneeangle= 0;
                Data.(combinedname).Ankleangle=0;
            else
                Data.(combinedname).Hipangle=Terials2unm(T2);
                Data.(combinedname).Kneeangle= Terials2unm(T2);
                Data.(combinedname).Ankleangle=30*3.14/180;
            end
             Refmmodel = Model(Data.RefModelpath);
             [osimmodel,Data.(combinedname).MuscleInfo]=Modelcreator(combinedname,Data,Refmmodel);
            
            for T3=1:length(Terials3)
                
                filename=append(psname,'_',upper(Data.whichleg),Joints(T1),'_',Terials2(T2),'_',Terials3(T3));
                Data.(filename).RefStatepath=append(Basepath,'\Data\',filename,'_Motion.mot');
                Data.(filename).RefControlpath=append(Basepath,'\Data\',filename,'_Torque.sto');
                Data.(filename).ParamSimulPath=append(Basepath,'\Parameterestimation\Parameter_Opt_',filename,'.sto');
                Data.(filename).ModelPath=append(Basepath,'\Model\',filename,'.osim');
                StateDataTable=TableProcessor(Data.(filename).RefStatepath);
                %                 HipAngle=StateDataTable.process.getDependentColumnAtIndex(0).getAsMat();
                %                 KneeAngle=StateDataTable.process.getDependentColumnAtIndex(1).getAsMat();
                %                 AnkleAngle=StateDataTable.process.getDependentColumnAtIndex(2).getAsMat();
                %                 BiodexAngle=StateDataTable.process.getDependentColumnAtIndex(3).getAsMat();
                %                 Data.(filename).Hipangle=mean(HipAngle);
                %                 Meanknee=mean(KneeAngle);
                %                 if Meanknee< 0 Meanknee=0; end
                %                 Data.(filename).Kneeangle= 1.57;
                %                 Data.(filename).Ankleangle=mean(AnkleAngle);
                
                Etimeindx=StateDataTable.process.getNumRows();
                
                ControlDataTable=TableProcessor(Data.(filename).RefControlpath);
                StateSolutionTable=StateDataTable.process;
                ControlSolutionTable=ControlDataTable.process;
                Data.Etime=double(StateSolutionTable.getIndependentColumn().get(Etimeindx-1));
                [kneeTrackingParamSolution]=ParameterEstimation(StateSolutionTable,ControlSolutionTable,osimmodel,combinedname,filename,Data);
                fprintf('Optimazation of %s is done \n',filename);
                osimmodel=changemodelproperty(osimmodel,filename,Data,1);
                qe=qe+1;
            end
        end
    end
end

save([Basepath '\SimData.mat'],'Data');
% Plotting()
function [data]= addingleg(data,whichleg)
for y=1:length(data)
    data(y)=append(data(y),"_",whichleg);
end
end