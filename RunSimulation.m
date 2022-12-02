clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)
SubjectNumber='T002';
Project='P006';
Basepath=append(['C:\MyCloud\OneDriveUcf\Real\Simulation\Source'],'\',Project,'\',SubjectNumber);
Pardata=importdata(append(Basepath,"\Data\Parameters.csv"));
Data.whichleg=string(extractBetween(Pardata.textdata{1},"=",","));
Data.joints=["ground_pelvis","hip","walker_knee","patellofemoral","ankle","mtp","subtalar"];
Data.joints(2:end)=addingleg(Data.joints(2:end),Data.whichleg);
Data.Weldjoints=["ground_pelvis","hip","mtp","subtalar"];
Data.Weldjoints(2:end)=addingleg(Data.Weldjoints(2:end),Data.whichleg);
Data.bodies=["pelvis","femur","tibia","patella","talus","calcn","toes"];
Data.bodies(2:end)=addingleg(Data.bodies(2:end),Data.whichleg);
% Make sure if you have common name in the Rigidtendon and Compliance
% Muscle put their name at the end of rigid tendon. 
Data.Rigidtendon=["bflh","bfsh","recfem","semimem","semiten","vasint","vaslat","vasmed"];
Data.Rigidtendon=addingleg(Data.Rigidtendon,Data.whichleg);
% Data.Rigidtendon=["gaslat_r","gasmed_r"];
% Data.ComplianacMusclename=["gaslat_r","gasmed_r"];
Data.ComplianacMusclename=[];
if Data.ComplianacMusclename
    Data.SimMusclename=[Data.Rigidtendon(~contains(Data.Rigidtendon,Data.ComplianacMusclename)),Data.ComplianacMusclename];
else
    Data.SimMusclename=Data.Rigidtendon;
end
Data.DeGrooteflage=1;
% Hipangle=0;%deg
% Basepath=[cd];
Data.optForce=Pardata.data(6);
Data.RefModelpath=append(Basepath,'\Model\',Project,'_',SubjectNumber,'_','Rajagopal_Scaled.osim');
% Data.RefStatepath=append(Basepath,'\TorqueSimulation\referenceCoordinates.sto');

Data.RefStatepathAnkleMoving=append(Basepath,'\TorqueSimulation\referenceCoordinatesAnkleMoving.sto');
Data.TorqueSolverinterval=40;
Data.ParamSolverinterval=50;
% Data.Etime=20;
Data.Stime=0;
Data.PassiveFiberBound=[0.05,0.9];
Data.TendonStrainBound=[0.01,0.1];
% Trialas=["KneeMove","AnkleMove","KneeAnkleMove"];
Trialas=["KneeMove"];
HipAngle=[90,70,55,40,25,10];
Kneeangle=[0];
Ankleangle=[0];

%% running just Parameter optimization
Data.justparameterflag=1;
qe=1;

for t=1:length(Trialas)
%     if contains(Trialas(t),"AnkleMove")       
%         tableProcessor=TableProcessor(Data.RefStatepathAnkleMoving);
%     else
%         Data.ActiveCoordinates=["knee_angle_r"];
%         tableProcessor=TableProcessor(Data.RefStatepath);
%     end
    for Hipindx=1:length(HipAngle)
        for kneeindx=1:length(Kneeangle)
            for ankleindx=1:length(Ankleangle)
                if contains(Trialas(t),"AnkleMove")
                    Data.Coordlable(qe)={append(Trialas(t),'_H',num2str(HipAngle(Hipindx)),'_K',num2str(Kneeangle(kneeindx)))};
                    Data.ActiveCoordinates=["ankle_angle"];
                    Data.ActiveCoordinates=addingleg(Data.ActiveCoordinates,Data.whichleg);
%                     Data.SimMusclename=["gaslat_r","gasmed_r"]
                elseif contains(Trialas(t),"KneeMove")
                    if Ankleangle(ankleindx)>=0
                        anklename=num2str(Ankleangle(ankleindx));
                    else
                        anklename=[num2str(abs(Ankleangle(ankleindx))),'Plan'];
                    end
                    Data.Coordlable(qe)={append(Trialas(t),'_H',num2str(HipAngle(Hipindx)),'_A',anklename)};
                    Data.ActiveCoordinates=["knee_angle"];
                    Data.ActiveCoordinates=addingleg(Data.ActiveCoordinates,Data.whichleg);
                end

                Data.(Data.Coordlable{qe}).Hipangle=HipAngle(Hipindx);
                Data.(Data.Coordlable{qe}).Kneeangle=Kneeangle(kneeindx);
                Data.(Data.Coordlable{qe}).Ankleangle=Ankleangle(ankleindx);
                lablell=append(Project,'_',SubjectNumber,'_',upper(Data.whichleg),'knee_Fl_H',num2str(HipAngle(Hipindx)));
                Data.(Data.Coordlable{qe}).RefStatepath=append(Basepath,'\Data\',lablell,'_Motion.mot');
                Data.(Data.Coordlable{qe}).RefControlpath=append(Basepath,'\Data\',lablell,'_Torque.mot');
                Data.(Data.Coordlable{qe}).TorqeSimulPath=append(Basepath,'\TorqueSimulation\Torque_Est_',Data.Coordlable{qe},'.sto');
                Data.(Data.Coordlable{qe}).ParamSimulPath=append(Basepath,'\Parameterestimation\Parameter_Opt_',Data.Coordlable{qe},'.sto');
                %         if Data.DeGrooteflage
                Data.(Data.Coordlable{qe}).ModelPath=append(Basepath,'\Model\',Project,'_',SubjectNumber,'_Model_',Data.Coordlable{qe},'.osim');
                %         else
                %             Data.(Data.Coordlable{qe}).ModelPath=append(Basepath,'\ModelGenerator\OneDOF_Knee_Thelen_',Data.Coordlable{qe},'.osim');
                %         end
                Refmmodel = Model(Data.RefModelpath);
                [osimmodel,Data.(Data.Coordlable{qe}).MuscleInfo]=Modelcreator(Data.Coordlable{qe},Data,Refmmodel);
                if ~Data.justparameterflag
                       StateDataTable=TableProcessor(Data.(Data.Coordlable{qe}).RefStatepath);
                       [kneeTrackingSolution]=TorqueSimulation(StateDataTable,osimmodel,Data.Coordlable{qe},Data);
                       [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolution.exportToStatesTable(),kneeTrackingSolution.exportToControlsTable(),osimmodel,Data.Coordlable{qe},Data);
                else
                    %DataTable=TableProcessor(Data.(Data.Coordlable{qe}).TorqeSimulPath);
                    StateDataTable=TableProcessor(Data.(Data.Coordlable{qe}).RefStatepath);
                    Etimeindx=StateDataTable.process.getNumRows();
                    ControlDataTable=TableProcessor(Data.(Data.Coordlable{qe}).RefControlpath);
                    StateSolutionTable=StateDataTable.process;
                    ControlSolutionTable=ControlDataTable.process;
                    Data.Etime=double(StateSolutionTable.getIndependentColumn().get(Etimeindx-1));
                    [kneeTrackingParamSolution]=ParameterEstimation(StateSolutionTable,ControlSolutionTable,osimmodel,Data.Coordlable{qe},Data);
                    osimmodel=changemodelproperty(osimmodel,Data.Coordlable{qe},Data,1);
                end
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