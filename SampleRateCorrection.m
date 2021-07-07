function [ResampledData]= SampleRateCorrection (Datadr,DStime)
Gdata=[0];
            data=importdata(Datadr);
            for row_num=2:2:length(data.textdata)
                lastdata_indx=1;
                tflage=0;
                firstdata_indx=0;
                for col_num=1:length(data.data)%# of data points in a given trial
                    if ~isnan(data.data(col_num,row_num))&&data.data(col_num,row_num)~=0
                        lastdata_indx=col_num;  %finding zero data at the end of each chanel
                        tflage=1;
                    elseif tflage==0
                        firstdata_indx=col_num+1;  %finding zero data at the begining
                    end
                end
                
                if row_num==2
                    ts=data.data(firstdata_indx,1); % first time of first chanel to set as final time for every other channel.
                    te=data.data(lastdata_indx,1); % final time of first chanel to set as final time for every other channel.
                end
                y=interp1(data.data(firstdata_indx:lastdata_indx,row_num-1),data.data(firstdata_indx:lastdata_indx,row_num),[ts:DStime:te],'linear','extrap'); %Interpolates data to match sampling time to desierd sampling time
                
                b=y';
                %                 ends(ii)=length(b);
                if (size(Gdata(:,1)) == 1) %recombines data into a matrix padded with NaN
                    Gdata = [[data.data(firstdata_indx,1):DStime:data.data(lastdata_indx,1)]' b];
                else
                    Gdata = [Gdata b];
                end
                
            end
            ResampledData.data=Gdata;
            ResampledData.colheaders=["time" data.textdata(2:2:end)];
            clear Gdata
            Gdata=[0];
                
end