clear all
% close all
import org.opensim.modeling.*;
Trc_path=['C:\MyCloud\OneDriveUcf\Real\Simulation\Passive_Parameter_prediction\Robotic_Leg\'];
listing = dir(Trc_path);
plotfalg=0;
IKflag=1;
Mnames=[];
Tnames=[];

% myLog = JavaLogSink();
% Logger.addSink(myLog);

runver="E3";
% SubjectNumber=["06","07","08","09","10","11","12","13","14","15"];
SubjectNumber=["10"];


SubjectNumber=append("T0",SubjectNumber);
Data.joints=["ground_pelvis","Hip_Joint","Knee_Joint","Ankle_Joint"];
Data.Weldjoints=["ground_pelvis","Ankle_Joint"];
Data.bodies=["Hip_Bone","Thigh","Shank","Foot"];
Data.maxpanlt=-deg2rad(15);
Data.DeGrooteflage=1;
Data.TorqueSolverinterval=40;
Data.ParamSolverinterval=50;
Data.PassiveFiberBound=[0.01,0.9];
Data.TendonStrainBound=[0.005,0.1];
Data.optForce=1;
psname=["Knee90Trial0","Knee45Trial0","Knee0Trial0"];
for S=length(psname)
    combinedname=psname(S);
    Data.(combinedname).ModelPath=fullfile(Trc_path,'Model',append(combinedname,'.osim'));
    Data.(combinedname).RefModelpath=fullfile(Trc_path,'Model','FinalModel.osim');
    Data.(combinedname).RefStatepath=fullfile(Trc_path,'Data',append(combinedname,'_IK.mot'));
    Data.(combinedname).RefControlpath=fullfile(Trc_path,'Data',append(combinedname,'_Torque.sto'));
    Data.(combinedname).Hipangle=deg2rad(90);
    Data.(combinedname).Kneeangle=0;
    Data.(combinedname).Ankleangle=0;
    Data.ActiveCoordinates=["Hip","Knee"];
    osimmodel = Model(Data.(combinedname).ModelPath);
%     osimmodel=Robotic_Modelcreator(combinedname,Data,Refmmodel);
    StateDataTable=TableProcessor(Data.(combinedname).RefStatepath).process;
    p=StateDataTable.getNumRows();
    Data.(combinedname).Stime=double(StateDataTable.getIndependentColumn().get(0));
    Data.(combinedname).Etime=double(StateDataTable.getIndependentColumn().get(p-1));
    [kneeTrackingSolution]=Roboticleg_TorqueSimulation(StateDataTable,osimmodel,combinedname,Data);
end
