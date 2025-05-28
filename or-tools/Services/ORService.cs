using Google.OrTools.ConstraintSolver;
using Google.Protobuf.WellKnownTypes;
using or_tools.Models;

namespace or_tools.Services
{
    public class ORService
    {
        public TspVrpResponse Solve(TspVrpRequest request)
        {
            var distanceMatrix = request.DistanceMatrix;
            int locationCount = distanceMatrix.Count;

            // Fallback if priorities not provided
            var priorities = request.Priorities ?? Enumerable.Repeat(1, locationCount).ToList();

            RoutingIndexManager manager = new(locationCount, 1, 0);
            RoutingModel routing = new(manager);

            int transitCallbackIndex = routing.RegisterTransitCallback((long fromIndex, long toIndex) =>
            {
                int fromNode = manager.IndexToNode(fromIndex);
                int toNode = manager.IndexToNode(toIndex);
                return distanceMatrix[fromNode][toNode];
            });

            routing.SetArcCostEvaluatorOfAllVehicles(transitCallbackIndex);

            // Add disjunctions based on priority
            for (int i = 1; i < locationCount; i++)
            {
                int priority = priorities[i];
                long penalty = priority switch
                {
                    2 => 10_000_000, // High priority – very costly to skip
                    1 => 10_000,    // Medium priority
                    0 => 1_000,     // Low priority – skippable
                    _ => 10_000
                };

                routing.AddDisjunction(new long[] { manager.NodeToIndex(i) }, penalty);
            }

            var searchParameters = operations_research_constraint_solver.DefaultRoutingSearchParameters();
            searchParameters.FirstSolutionStrategy = FirstSolutionStrategy.Types.Value.Automatic;
            searchParameters.LocalSearchMetaheuristic = LocalSearchMetaheuristic.Types.Value.SimulatedAnnealing;
            //searchParameters.TimeLimit = new Duration { Seconds = 20 };
            searchParameters.TimeLimit = new Duration { Seconds = 5 };
            searchParameters.UseFullPropagation = true;

            Assignment solution = routing.SolveWithParameters(searchParameters);
            if (solution == null) return null;

            List<int> route = new();
            long totalDistance = 0;
            var index = routing.Start(0);

            while (!routing.IsEnd(index))
            {
                int nodeIndex = manager.IndexToNode(index);
                route.Add(nodeIndex);
                var nextIndex = solution.Value(routing.NextVar(index));
                totalDistance += routing.GetArcCostForVehicle(index, nextIndex, 0);
                index = nextIndex;
            }

            route.Add(manager.IndexToNode(index));

            return new TspVrpResponse { Route = route, Distance = totalDistance };
        }
    }
}
