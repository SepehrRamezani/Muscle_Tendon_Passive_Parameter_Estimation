clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)

% Data.SimMusclename=["bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
Data.SimMusclename=["gaslat_r","gasmed_r"];
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
% Trialas=["Hip","AnkleMovingHip"];
Trialas=["AnkleMovingHip"];
HipAngle=[90];
%% running just Parameter optimization
Data.justparameterflag=0;
qe=1;
for t=1:length(Trialas)
    if contains(Trialas(t),"AnkleMoving")
        Data.ActiveCoordinates=["ankle_angle_r"];
        tableProcessor=TableProcessor(Data.RefStatepathAnkleMoving);
    else
        Data.ActiveCoordinates=["knee_angle_r"];
        tableProcessor=TableProcessor(Data.RefStatepath);
    end
    for ce=1:length(HipAngle)
        Data.Hipangle(qe)=HipAngle(ce);
        Data.Hiplable(qe)={append(Trialas(t),num2str(HipAngle(ce)))};
        Data.(Data.Hiplable{qe}).TorqeSimulPath=append(cd,'\TorqueSimulation\Kneeflexion_solution_Degroot_',Data.Hiplable{qe},'.sto');
        Data.(Data.Hiplable{qe}).ParamSimulPath=append(cd,'\Parameterestimation\Parameter_Opt_',Data.Hiplable{qe},'.sto');
%         if Data.DeGrooteflage
        Data.(Data.Hiplable{qe}).ModelPath=append(cd,'\ModelGenerator\OneDOF_Knee_DeGroote_',Data.Hiplable{qe},'.osim');
%         else
%             Data.(Data.Hiplable{qe}).ModelPath=append(cd,'\ModelGenerator\OneDOF_Knee_Thelen_',Data.Hiplable{qe},'.osim');
%         end
        Refmmodel = Model(Data.RefModelpath);
        [osimmodel,Data.(Data.Hiplable{qe}).MuscleInfo]=Modelcreator(HipAngle(ce),Data.Hiplable{qe},Data,Refmmodel);
            if ~Data.justparameterflag
        
                [kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Data.Hiplable{ce},Data);
                [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolution.exportToStatesTable(),kneeTrackingSolution.exportToControlsTable(),osimmodel,Data.Hiplable{ce},Data);
            else
                DataTable=TableProcessor(Data.(Data.Hiplable{ce}).TorqeSimulPath);
                kneeTrackingSolutionTable=DataTable.process;
                [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolutionTable,kneeTrackingSolutionTable,osimmodel,Data.Hiplable{ce},Data);
            end
        qe=qe+1;
    end
end
save([cd '\SimData.mat'],'Data');
Plotting()
