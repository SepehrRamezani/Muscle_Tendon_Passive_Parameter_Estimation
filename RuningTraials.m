clear all
import org.opensim.modeling.*
Logger.addSink(JavaLogSink())
path='C:\Program Files\OpenSim 4.1\Geometry';
ModelVisualizer.addDirToGeometrySearchPaths(path);
%% File address %%
folder = 'C:\MyCloud\OneDriveUcf\Real\Simulation\Source\P006\T002\';
psname='P006_T002_';
DynamicMarkerFile='New_subject01_walk1.trc';
Resultdir='\Result\CMC\Rajagopal\';
genericSetupForScale='subject01_Setup_Scale.xml';
AnalyzeMethod=["SOP","CMC"];
Modelname=["Rajagopal"];
load (append(folder,"Result\",psname,"ResultData.mat"));
%% Runing simulation
TimeT=zeros(4,2);
filename=ResultData.info.trialsname;
for m=1:length(Modelname)
    results_folder = append(folder,"Result\",Modelname(m),"\");
    modeldir=append(folder,'Model\',Modelname(m),'\',psname,Modelname(m),"_Scaled.osim");
    status = mkdir(append(results_folder,"ID\"));
    %   IK_file=append(results_folder,name,'_ik.mot');
    %   NewExForcesetup=append(results_folder,'New_subject01_walk1_grf.xml');
    CMCSetup=append('CMC\',Modelname(m),'\CMC_Setup_I0.xml');
    IDSetup=append('CMC\',Modelname(m),'\ID_Setup.xml');
    for T1=4:length(filename)
        if contains(filename(T1),"Q") & T1~=22
            Header=filename(T1);
            MotionData=ResultData.(Header).Motion;
            ExLoad=ExternalLoads(append(results_folder,psname,"ExForce_RLeg.xml"),true);
            ExForcefile=append(folder,"Data\",psname,Header,"_Torque.mot");
            ExLoad.setDataFileName(ExForcefile);
            NewExForcefile=append(results_folder,"ID\",Header,"_ExForce_Setup.xml");
            ExLoad.print(NewExForcefile)
            IkFile=append(folder,"Data\",psname,Header,"_Motion.mot");
            Stime=ResultData.(Header).Events(1);
            Etime=ResultData.(Header).Events(2);
            
             %% Doing some changes in Model
            HipFl=mean(MotionData([1:500],3));
            osimModel=Model(modeldir);
            osimModel.setName(append('Scaled_model_',Header));
            modelCoordSet = osimModel.getCoordinateSet();
            currentcoord = modelCoordSet.get(0);
            currentcoord.setDefaultValue(pi/2-HipFl);
            currentcoord1 = modelCoordSet.get(6);
            currentcoord1.setDefaultValue(HipFl);
%             currentcoord1.setLocked(true);
%             osimModel.print(append(results_folder,Header,".osim"));
            %% ID %%%%
            idTool=	InverseDynamicsTool(append(results_folder,psname,"ID_Setup_ref.xml"));
            idTool.setModel(osimModel);
            idTool.setModelFileName(append(results_folder,Header,".osim"))
            idTool.setStartTime(Stime);
            idTool.setEndTime(Etime);
            idTool.setCoordinatesFileName(IkFile);
            idTool.setExternalLoadsFileName(NewExForcefile);
            idTool.setResultsDir(append(results_folder,"ID\"))
            idTool.setOutputGenForceFileName(append(Header,"_ID.sto"))
%              idTool.print(append(results_folder,"ID\",filename,"_ID_Setup.xml"));
            idTool.run();
            %% SOP and CMC
            results_folder2=append(results_folder,AnalyzeMethod,"\",Header,"\");
            status = mkdir(results_folder2(1));
            status = mkdir(results_folder2(2));
            %% SOP %%%%%
            analysis = AnalyzeTool(append(results_folder,psname,"SOP_Setup_ref.xml"));
            analysis.setModel(osimModel);
            analysis.setModelFilename(append(results_folder,Header,".osim"))
            analysis.setName(append(Modelname(m),'_',Header))
            analysis.setInitialTime(Stime(1));
            analysis.setFinalTime(Etime(1));
            analysis.setLowpassCutoffFrequency(6);
            analysis.setCoordinatesFileName(IkFile);
            analysis.setExternalLoadsFileName(NewExForcefile);
            analysis.setLoadModelAndInput(true);
            analysis.setResultsDir(append(results_folder2(1)));
            analysis.print(append(results_folder2(1),Header,"_",AnalyzeMethod(1),"_Setup.xml"))
            Logger.addSink(JavaLogSink())
            analysis.run();
            %% CMC %%%%%
            cmc = CMCTool(append(results_folder,psname,"CMC_Setup_ref.xml"));
            cmc.setModel(osimModel);
            cmc.setModelFilename(append(results_folder,Header,".osim"))
            cmc.setName(append(Modelname(m),'_',Header,'_CMC'))
            cmc.setDesiredKinematicsFileName(IkFile);
            cmc.setLowpassCutoffFrequency(6);
            cmc.setExternalLoadsFileName(NewExForcefile);
            cmc.setStartTime(Stime(1));
            cmc.setFinalTime(Etime(1));
            cmc.setResultsDir(append(results_folder2(2)));
%             cmc.print(append(results_folder2(2),Header,"_",AnalyzeMethod(2),"_Setup.xml"));
%             cmc.run();
            clear cmc ExLoad idTool analysis
            %                 end
        end
    end
end


% look at AbstractTool () to find more subclass for time range

