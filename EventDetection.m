function Events = EventDetection(Trialname,Time,MTable)
if contains(Trialname,"Knee")
%% finding start time
DMCounter=0;
Thereshold=0.006;
DM=diff(MTable);
DAVG= mean(MTable(10:200));
[Sindx,c]=find(MTable >= DAVG + Thereshold);
Stime=Time(Sindx(1));
%% finding end time
Eindx=[];
for d=Sindx:length(DM)
    if DM(d-1)==0 & DM(d)==0 
        DMCounter=DMCounter+1;
    else
        DMCounter=0;
    end
    if DMCounter>20
        Eindx=[Eindx d-DMCounter];
    end
end
end  
Etime=Time(Eindx(1));
Events=[Stime,Etime];
end