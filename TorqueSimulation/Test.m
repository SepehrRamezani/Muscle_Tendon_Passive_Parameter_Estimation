clear all
import org.opensim.modeling.*;
Logger.addSink(JavaLogSink());
osismmodel = Model('subject_walk_armless.osim');
osismmodel.finalizeConnections();
DeGrooteFregly2016Muscle().replaceMuscles(osismmodel);
SimMusclename=["bflh_r","bfsh_r","gaslat_r"]
c=0;
for m = 0:osismmodel.getMuscles().getSize()-1
        musc = osismmodel.updMuscles().get(c);
        if ~sum(strcmp(char(musc.getName()), SimMusclename))
        osismmodel.updMuscles().remove(c)
        osismmodel.initSystem();
        else
            c=c+1;
        end
end
osismmodel.print('subject_walk_armless_DeGroote.osim');