function [Gdata,Gheader]= ReshapingData(data,DStime)
Gdata=[0];
Gheader=[];
timeinx=find(contains(data.textdata,"X [s]"));

for ii=1:length(timeinx)
    kk=1;
    tflage=0;
    ww=0;
    for jj=1:length(data.data)%# of data points in a given trial
        if ~isnan(data.data(jj,timeinx(ii)+1))&&data.data(jj,timeinx(ii)+1)~=0
            kk=jj;  %finding zero data at the end of each chanel
            tflage=1;
        elseif tflage==0
            ww=jj;  %finding zero data at the begining
        end
    end
    ts(ii)=data.data(ww+1,timeinx(ii)); % first time of first chanel to set as final time for every other channel.
    te(ii)=data.data(kk,timeinx(ii)); % final time of first chanel to set as final time for every other channel.
end
%%             finding the highest starting
tsfinal=max(ts);
tefinal=min(te);
interpolatetime=tsfinal:DStime:tefinal;
for ii=1:length(timeinx)
    
    starttimeinx=find(data.data(:,timeinx(ii))>= tsfinal & data.data(:,timeinx(ii))<=tefinal);
    if ii==length(timeinx)
        y=interp1(data.data(starttimeinx,timeinx(ii)),data.data(starttimeinx,[timeinx(ii)+1:end]),interpolatetime,'linear','extrap'); %Interpolates data to match sampling time to desierd sampling time
    else
        y=interp1(data.data(starttimeinx,timeinx(ii)),data.data(starttimeinx,[timeinx(ii)+1:timeinx(ii+1)-1]),interpolatetime,'linear','extrap'); %Interpolates data to match sampling time to desierd sampling time
    end
    [ry,cy]=size(y);
    if ry==1
        b=y';
    else
        b=y;
    end
    %                 ends(ii)=length(b);
    if (size(Gdata(:,1)) == 1) %recombines data into a matrix padded with NaN
        Gdata = [interpolatetime' b];
        if ii==length(timeinx)
            Gheader=["time" data.textdata([timeinx(ii)+1:end])];
        else
            Gheader=["time" data.textdata([timeinx(ii)+1:timeinx(ii+1)-1])];
        end
    else
        Gdata = [Gdata b];
        if ii==length(timeinx)
            Gheader=[Gheader data.textdata(timeinx(ii)+1:end)];
        else
            Gheader=[Gheader data.textdata(timeinx(ii)+1:timeinx(ii+1)-1)];
        end
    end
  
end



end