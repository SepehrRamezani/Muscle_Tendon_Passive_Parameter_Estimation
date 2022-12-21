clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)
SubjectNumber='T006';
Project='P006';
psname=append(Project,'_',SubjectNumber);

Basepath=append(['C:\MyCloud\OneDriveUcf\Real\Simulation\Source'],'\',Project,'\',SubjectNumber);
Pardata=importdata(append(Basepath,"\Data\Parameters.csv"));
Data.whichleg=string(extractBetween(Pardata.textdata{1},"=",","));
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
Data.optForce=Pardata.data(6);
Data.RefModelpath=append(Basepath,'\Model\',psname,'_Rajagopal_Scaled.osim');
% Data.RefStatepath=append(Basepath,'\TorqueSimulation\referenceCoordinates.sto');

Data.RefStatepathAnkleMoving=append(Basepath,'\TorqueSimulation\referenceCoordinatesAnkleMoving.sto');
Data.TorqueSolverinterval=40;
Data.ParamSolverinterval=50;
% Data.Etime=20;
Data.Stime=0;
Data.PassiveFiberBound=[0.05,0.9];
Data.TendonStrainBound=[0.01,0.1];

% Trialas=["KneeMove","AnkleMove","KneeAnkleMove"];
Joints=["Ankle","Knee"];
Terials3=["L1","L2","L3"];
% Trialas=["KneeMove"];
% HipAngle=[90,70,55,40,25,10];
% Kneeangle=[0];
% Ankleangle=[0];

%% running just Parameter optimization
Data.justparameterflag=1;
qe=1;

for T1=1:length(Joints)
        if contains(Joints(T1),"Knee")
%             Terials2=["H90","H55","H15"];
            Terials2=["H90"];
            Data.ActiveCoordinates=["knee_angle"];
            Data.ActiveCoordinates=addingleg(Data.ActiveCoordinates,Data.whichleg);
            Data.Rigidtendon=["bflh","bfsh","recfem","semimem","semiten","vasint","vaslat","vasmed"];
            Data.Rigidtendon=addingleg(Data.Rigidtendon,Data.whichleg);
            Data.ComplianacMusclename=[];
        else
            %             Terials2=["K90","K45","K0"];
            Terials2=["K90"];
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
            for T3=1:length(Terials3)
                
                filename=append(psname,'_',upper(Data.whichleg),Joints(T1),'_',Terials2(T2),'_',Terials3(T3));

                Data.(filename).RefStatepath=append(Basepath,'\Data\',filename,'_Motion.mot');
                Data.(filename).RefControlpath=append(Basepath,'\Data\',filename,'_Torque.mot');
                Data.(filename).TorqeSimulPath=append(Basepath,'\TorqueSimulation\Torque_Est_',filename,'.sto');
                Data.(filename).ParamSimulPath=append(Basepath,'\Parameterestimation\Parameter_Opt_',filename,'.sto');
                Data.(filename).ModelPath=append(Basepath,'\Model\',psname,'_Model_',filename,'.osim');
                
                StateDataTable=TableProcessor(Data.(filename).RefStatepath);
                HipAngle=StateDataTable.process.getDependentColumnAtIndex(0).getAsMat();
                KneeAngle=StateDataTable.process.getDependentColumnAtIndex(1).getAsMat();
                AnkleAngle=StateDataTable.process.getDependentColumnAtIndex(2).getAsMat();
                BiodexAngle=StateDataTable.process.getDependentColumnAtIndex(3).getAsMat();
                
                Data.(filename).Hipangle=mean(HipAngle);
                Meanknee=mean(KneeAngle);
                if Meanknee< 0 Meanknee=0; end
                Data.(filename).Kneeangle= 1.57;
                Data.(filename).Ankleangle=mean(AnkleAngle);
                Etimeindx=StateDataTable.process.getNumRows();
                
                Refmmodel = Model(Data.RefModelpath);
                [osimmodel,Data.(filename).MuscleInfo]=Modelcreator(filename,Data,Refmmodel);
                
                
                ControlDataTable=TableProcessor(Data.(filename).RefControlpath);
                StateSolutionTable=StateDataTable.process;
                ControlSolutionTable=ControlDataTable.process;
                Data.Etime=double(StateSolutionTable.getIndependentColumn().get(Etimeindx-1));
                [kneeTrackingParamSolution]=ParameterEstimation(StateSolutionTable,ControlSolutionTable,osimmodel,filename,Data);
                osimmodel=changemodelproperty(osimmodel,filename,Data,1);
                qe=qe+1;
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