package bearmaps.hw4;
import bearmaps.proj2ab.ExtrinsicMinPQ;
import bearmaps.proj2ab.ArrayHeapMinPQ;
import edu.princeton.cs.algs4.Stopwatch;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class AStarSolver<Vertex> implements ShortestPathsSolver<Vertex> {

    private ExtrinsicMinPQ<Vertex> fringe = new ArrayHeapMinPQ<>();
    private int numStatesExplored = 0;
    private Map<Vertex, Double> distTo = new HashMap<>();
    private Map<Vertex, Boolean> visited = new HashMap<>();
    private double endTime;
    private double maxTime;
    private List<Vertex> tempSolution = new ArrayList<>();
    private List<Vertex> realSolution = new ArrayList<>();
    private Map<Vertex, Vertex> edgeTo = new HashMap<>();
    private double solutionWeight;
    private double explorationTime;
    private boolean condition;

    public AStarSolver(AStarGraph<Vertex> input, Vertex start, Vertex end, double timeout) {
        Stopwatch timer = new Stopwatch();
        maxTime = timeout;
        distTo.put(start, 0.0);
        edgeTo.put(start, null);
        double minWeight = input.estimatedDistanceToGoal(start, end);
        fringe.add(start, minWeight);
        Vertex smallest = fringe.getSmallest();

        if (start.equals(end)) {
            edgeTo.put(start, null);
            distTo.put(start, 0.0);
        }
        while ((!smallest.equals(end)) && (timer.elapsedTime() <= timeout) && (fringe.size() > 0)) {
            Vertex v = fringe.removeSmallest();
            visited.put(v, true);
            numStatesExplored++;
            for (WeightedEdge w: input.neighbors(v)) {
                if (!visited.containsKey(w.to())) {
                    distTo.put((Vertex) w.to(), Double.POSITIVE_INFINITY);
                    visited.put((Vertex) w.to(), false);
                }
            }
            for (WeightedEdge w: input.neighbors(v)) {
                relax(input, w, end);
            }
            if (fringe.size() < 1) {
                break;
            } else {
                smallest = fringe.getSmallest();
            }

        }
        condition = timer.elapsedTime() >= timeout;
        if (condition) {
            return;
        }
        if (outcome().equals(SolverOutcome.SOLVED)) {
            Vertex x = end;
            while (x != null) {
                tempSolution.add(x);
                x = edgeTo.get(x);
            }
            int s = tempSolution.size();
            for (int i = s - 1; i >= 0; i--) {
                Vertex y = tempSolution.remove(i);
                realSolution.add(y);
            }
            solutionWeight = distTo.get(end);
        }
        explorationTime = timer.elapsedTime();

    }
    public SolverOutcome outcome() {
        if (condition) {
            return SolverOutcome.TIMEOUT;
        } else if (fringe.size() == 0) {
            return SolverOutcome.UNSOLVABLE;
        } else {
            return SolverOutcome.SOLVED;
        }
    }
    public List<Vertex> solution() {
        if (outcome().equals(SolverOutcome.UNSOLVABLE) || outcome().equals(SolverOutcome.TIMEOUT)) {
            return null;
        } else {
            return realSolution;
        }
    }
    public double solutionWeight() {
        if (outcome().equals(SolverOutcome.UNSOLVABLE) || outcome().equals(SolverOutcome.TIMEOUT)) {
            return 0;
        }
        return solutionWeight;
    }
    public int numStatesExplored() {
        return numStatesExplored;
    }
    public double explorationTime() {
        return explorationTime;
    }

    private void relax(AStarGraph<Vertex> input, WeightedEdge w, Vertex last) {
        Vertex p = (Vertex) w.from();
        Vertex q = (Vertex) w.to();
        if (!visited.get(q)) {
            double weight = w.weight();
            double x = distTo.get(p) + weight;
            double h = input.estimatedDistanceToGoal(q, last);
            if (x < distTo.get(q)) {
                distTo.put(q, x);
                double newPriority = distTo.get(q) + h;
                if (fringe.contains(q)) {
                    fringe.changePriority(q, newPriority);
                } else {
                    fringe.add(q, newPriority);
                }
                edgeTo.put(q, p);
            }
        }
    }

}
