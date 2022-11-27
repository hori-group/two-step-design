classdef SummaryOfSimulationData
   properties
      filename
      Cost_runs_list = zeros(1,100) % Cost_filename
      ff_runs_list = zeros(1,100) % ff_filename
      Cost_PoI_runs_list = zeros(1,100) % Cost_filename_s
%      Cost_plus_500 = zeros(1,100) 
      N_PoI % N_filename_s
      N_PoS % N_ff_filename
      mean_cost_all_runs 
      
   end
   methods
      function r = roundOff(obj)
         r = round([obj.Value],2);
      end
   end
end



















