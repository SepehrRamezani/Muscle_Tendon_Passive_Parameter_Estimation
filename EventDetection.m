function Events = EventDetection(filename,DStime,MTable,Threshold)
if contains(filename,"RKnee")
    ref = mean(MTable(1:200,5));
    MTable(:,5)=MTable(:,5)-ref;
    [bb,aa] = butter(4, 0.02,'low');
    datafilt=filter(bb,aa,MTable(:,5));
    Biodexfilterdmotion=filter(bb,aa,datafilt);
    % & [0;diff(MTable(:,6))>0]
    [indx,c]=find(abs(diff(Biodexfilterdmotion))>=0.0006);
    %% we some constatnt data befor knee flexes and 200 is for that
    Sindx=indx(1)-200;
    %% 97 is a delayed samle because of filtering 
    Eindx=indx(end)-97;
    Stime=MTable(Sindx,1);
    Etime=MTable(Eindx,1);
%     count=0;
%     Stime=[];
%     Etime=[];
%     for ww=1:length(Sindx)
%         if (MTable(Sindx(ww))-MTable(Sindx(ww)-1))>0
%             count=count+1;
%             Stime=[Stime;Time(Sindx(ww),1)];
%             Etime=[Etime;Time(Eindx(ww),1)];
%         end
%     end
    
    %     [indx,c]=find(abs(FTable)>ForceRatio.*max(abs(FTable)));
    %     Stime=Time(indx([1;find(diff(indx)>10)+1]),1);
    %     %Etime=Time(indx([(find(diff(indx)>10));end]),1);
    %     Etime=Stime+3;
elseif contains(filename,"RAnkle")
    if contains(filename,"Fl")
        [indx,c]=find(MTable>=Threshold(1) & MTable<=Threshold(2));
        % & [0;diff(MTable(:,6))>0]
        Sindx=indx([1;find(diff(indx)>10)+1]);
        Eindx=indx([(find(diff(indx)>10));end]);
        count=0;
        Stime=[];
        Etime=[];
        for ww=1:length(Sindx)
            if (MTable(Sindx(ww))-MTable(Sindx(ww)-1))>0
                count=count+1;
                Stime=[Stime;Time(Sindx(ww),1)];
                Etime=[Etime;Time(Eindx(ww),1)];
            end
        end
    else
        [indx,c]=find(MTable>=Threshold(1) & MTable<=(Threshold(2)-5/180*3.14));
        Sindx=indx([1;find(diff(indx)>10)+1]);
        Eindx=indx([(find(diff(indx)>10));end]);
        count=0;
        Stime=[];
        Etime=[];
        for ww=1:length(Sindx)
            if (MTable(Sindx(ww)+1)-MTable(Sindx(ww)))<0
                count=count+1;
                Stime=[Stime;Time(Sindx(ww),1)];
                Etime=[Etime;Time(Eindx(ww),1)];
            end
        end
    end
else
    fprintf('\nERROR: %s unknown trail ...\n\n', filename);
end
Events=[Sindx,Eindx];
end