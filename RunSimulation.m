clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)

Data.SimMusclename=["bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
% Data.SimMusclename=["gaslat_r","gasmed_r"];
Data.ComplianacMusclename=["gaslat_r","gasmed_r"];
Data.DeGrooteflage=1;
% Hipangle=0;%deg
Data.RefModelpath=append(cd,'\ModelGenerator\subject_walk_armless_RLeg_justknee.osim');
Data.RefStatepath=append(cd,'\TorqueSimulation\referenceCoordinates.sto');
Data.RefStatepathAnkleMoving=append(cd,'\TorqueSimulation\referenceCoordinatesAnkleMoving.sto');

Data.TorqueSolverinterval=40;
Data.ParamSolverinterval=40;
Data.Etime=20;
Data.Stime=0;
% Trialas=["KneeMove","AnkleMove","KneeAnkleMove"];
Trialas=["KneeMove"];
HipAngle=[90,45,0];
Kneeangle=[0];
Ankleangle=[0, -25,25];
%% running just Parameter optimization
Data.justparameterflag=0;
qe=1;

for t=1:length(Trialas)
%     if contains(Trialas(t),"AnkleMove")
        
%         tableProcessor=TableProcessor(Data.RefStatepathAnkleMoving);
%     else
%         Data.ActiveCoordinates=["knee_angle_r"];
        tableProcessor=TableProcessor(Data.RefStatepath);
%     end
    for Hipindx=1:length(HipAngle)
        for kneeindx=1:length(Kneeangle)
            for ankleindx=1:length(Ankleangle)
                if contains(Trialas(t),"AnkleMove")
                    Data.Coordlable(qe)={append(Trialas(t),'_H',num2str(HipAngle(Hipindx)),'_K',num2str(Kneeangle(kneeindx)))};
                    Data.ActiveCoordinates=["ankle_angle_r"];
                elseif contains(Trialas(t),"KneeMove")
                    if Ankleangle(ankleindx)>=0
                        anklename=num2str(Ankleangle(ankleindx));
                    else
                        anklename=[num2str(abs(Ankleangle(ankleindx))),'Plan'];
                    end
                    Data.Coordlable(qe)={append(Trialas(t),'_H',num2str(HipAngle(Hipindx)),'_A',anklename)};
                    Data.ActiveCoordinates=["knee_angle_r"];
                end
                Data.(Data.Coordlable{qe}).Hipangle=HipAngle(Hipindx);
                Data.(Data.Coordlable{qe}).Kneeangle=Kneeangle(kneeindx);
                Data.(Data.Coordlable{qe}).Ankleangle=Ankleangle(ankleindx);
                Data.(Data.Coordlable{qe}).TorqeSimulPath=append(cd,'\TorqueSimulation\Torque_Est_',Data.Coordlable{qe},'.sto');
                Data.(Data.Coordlable{qe}).ParamSimulPath=append(cd,'\Parameterestimation\Parameter_Opt_',Data.Coordlable{qe},'.sto');
                %         if Data.DeGrooteflage
                Data.(Data.Coordlable{qe}).ModelPath=append(cd,'\ModelGenerator\Model_',Data.Coordlable{qe},'.osim');
                %         else
                %             Data.(Data.Coordlable{qe}).ModelPath=append(cd,'\ModelGenerator\OneDOF_Knee_Thelen_',Data.Coordlable{qe},'.osim');
                %         end
                Refmmodel = Model(Data.RefModelpath);
                [osimmodel,Data.(Data.Coordlable{qe}).MuscleInfo]=Modelcreator(Data.Coordlable{qe},Data,Refmmodel);
                if ~Data.justparameterflag
                    
%                       [kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Data.Coordlable{qe},Data);
%                       [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolution.exportToStatesTable(),kneeTrackingSolution.exportToControlsTable(),osimmodel,Data.Coordlable{qe},Data);
                else
                    DataTable=TableProcessor(Data.(Data.Coordlable{qe}).TorqeSimulPath);
                    kneeTrackingSolutionTable=DataTable.process;
                    [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolutionTable,kneeTrackingSolutionTable,osimmodel,Data.Coordlable{qe},Data);
                end
                qe=qe+1;
            end
        end
    end
end
save([cd '\SimData.mat'],'Data');
% Plotting()
