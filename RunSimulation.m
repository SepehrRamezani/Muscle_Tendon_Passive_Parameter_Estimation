clear all
import org.opensim.modeling.*;
myLog = JavaLogSink();
Logger.addSink(myLog)
Data.SimMusclename=["knee_act","bflh_r","bfsh_r","gaslat_r","gasmed_r","recfem_r","semimem_r","semiten_r","vasint_r","vaslat_r","vasmed_r"];
Data.ComplianacMusclename=["gaslat_r","gasmed_r"];
Data.DeGrooteflage=1;
% Hipangle=0;%deg
Data.RefModelpath=append(cd,'\ModelGenerator\subject_walk_armless_RLeg_justknee.osim');
Data.RefStatepath=append(cd,'\TorqueSimulation\referenceCoordinates.sto');
tableProcessor=TableProcessor(Data.RefStatepath);
Data.TorqueSolverinterval=20;
Data.ParamSolverinterval=40;
Data.Etime=20;
Data.Stime=0;
Data.justparameter=1;
ce=0;
for Hipangle=0:45:90
    Hipangle=90-Hipangle;
    ce=ce+1;
    Data.Hipangle(ce)=Hipangle;
    Hiplable=append('Hip',num2str(Hipangle));
    Data.(Hiplable).SimulPath=[cd '\TorqueSimulation\Kneeflexion_solution_Degroot_' Hiplable '.sto'];
    if Data.DeGrooteflage
        Data.(Hiplable).ModelPath=[cd '\ModelGenerator\OneDOF_Knee_DeGroote_' Hiplable '.osim'];
    else
        Data.(Hiplable).ModelPath=[cd '\ModelGenerator\OneDOF_Knee_Thelen_' Hiplable '.osim'];
    end
    Refmmodel = Model(Data.RefModelpath);
    [osimmodel,Data.(Hiplable).MuscleInfo]=Modelcreator(Hipangle,Data,Refmmodel);
    if ~Data.justparameter
        
        [kneeTrackingSolution]=TorqueSimulation(tableProcessor,osimmodel,Hiplable,Data);
        [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolution.exportToStatesTable(),kneeTrackingSolution.exportToControlsTable(),osimmodel,Hiplable,Data);
    else
        DataTable=TableProcessor(Data.(Hiplable).SimulPath);
        kneeTrackingSolutionTable=DataTable.process;
        [kneeTrackingParamSolution]=ParameterEstimation(kneeTrackingSolutionTable,kneeTrackingSolutionTable,osimmodel,Hiplable,Data);
    end
end
save([cd '\SimData.mat'],'Data');
% Save(Data)
% bar(Error)
Plotting()
