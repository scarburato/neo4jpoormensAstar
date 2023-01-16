package londonSafeTravel;

import org.neo4j.graphdb.*;
import org.neo4j.procedure.*;
import org.neo4j.values.storable.PointValue;

import java.util.HashMap;
import java.util.PriorityQueue;
import java.util.stream.Stream;

public class RoutingAStart {
    static final double MPH_TO_MS = 0.44704;
    static final Label POINT = Label.label("Point");
    static final RelationshipType CONNECTS = RelationshipType.withName("CONNECTS");

    static final HashMap<String, Double> timeWeights = new HashMap<>() {{
        put("primary", 1.2);
        put("secondary", 1.4);
        put("tertiary", 1.6);
        put("unclassified", 1.8);
        put("road", 1.8);
        put("residential", 2.0);
        put("living_street", 2.4);
        put("service", 2.4);
    }};

    //@Context
    //public Log log;

    @Procedure(value = "londonSafeTravel.route", mode = Mode.READ)
    @Description("Finds the optimal path between two POINTS")
    public Stream<HopRecord> route(
            @Name("start") Node start,
            @Name("end") Node end,
            @Name("crossTimeField") String crossTimeField,
            @Name("maxSpeed") double maxspeed
    ) {

        if(!start.hasLabel(POINT))
            throw new IllegalArgumentException("`start' does not have `Point' as a label");

        if(!end.hasLabel(POINT))
            throw new IllegalArgumentException("`end' does not have `Point' as a label");

        PriorityQueue<RouteNode> openSetHeap = new PriorityQueue<>();
        HashMap<Long, RouteNode> openSet = new HashMap<>();
        HashMap<Long, RouteNode> closedSet = new HashMap<>();

        var startNode = new RouteNode(new Cost(0, heuristic(start, end,maxspeed)), start);
        openSetHeap.add(startNode);
        openSet.put(startNode.getId(), startNode);

        RouteNode current = null;
        while(! openSet.isEmpty()) {
            assert (openSet.size() == openSetHeap.size());

            // Get current node
            current = openSetHeap.poll();
            openSet.remove(current.getId());

            // Move from open to closed
            closedSet.put(current.getId(), current);

            // Found the solution HALT!
            if(current.getId() == getIdOf(end))
                break;

            var hops = current.node.getRelationships(Direction.OUTGOING, CONNECTS);

            for(var way : hops) {
                // NO ENTRY IN HERE!
                final double crossTimeStored = (Double) way.getProperty(crossTimeField);
                if(crossTimeStored == Double.POSITIVE_INFINITY)
                    continue;

                // Successor info
                Node successor = way.getEndNode();
                final long successorId = getIdOf(successor);

                // cost for this edge
                double crossTime = crossTime(way, current.node, successor,maxspeed);

                // Adjust for minor roads
                if(crossTimeField.equals("crossTimeMotorVehicle")) {
                    Double factor = timeWeights.get((String)way.getProperty("class"));
                    if(factor != null)
                        crossTime *= factor;
                }

                // If big intersection, add 5 seconds
                if(successor.getDegree(CONNECTS, Direction.INCOMING) > 3)
                    crossTime += 2.0;

                // cost function
                double travelTime = current.cost.g + crossTime;

                // if in open list and g' < g
                var in_open = openSet.get(successorId);
                if(in_open != null && travelTime < in_open.cost.g) {
                    // Iterate over the heap and find it
//                    RouteNode target = null;
//                    for(RouteNode t : openSetHeap) {
//                        if(t.getId() != successorId)
//                            continue;
//
//                        target = t;
//                        break;
//                    }
//
//                    assert (target != null);

                    openSetHeap.remove(in_open);
                    openSet.remove(successorId);

                    in_open = null;
                }

                // if in closed and g' < g
                var in_close = closedSet.get(successorId);
                if(in_close != null && travelTime < in_close.cost.g) {
                    closedSet.remove(successorId);
                    in_close = null;
                }

                // We found a better path OR We first visited this node!
                if(in_close == null && in_open == null) {
                    Cost newCost = new Cost(travelTime, heuristic(successor, end, maxspeed));
                    var newNode = new RouteNode(newCost, successor, current.getId());

                    openSet.put(successorId, newNode);
                    openSetHeap.add(newNode);
                }
            }
        }

        if(current.getId() != getIdOf(end))
            return Stream.<HopRecord>builder().build();

        long i = 0;
        Stream.Builder<HopRecord> hops = Stream.builder();

        for(
                RouteNode step = current;
                step.getId() != getIdOf(start);
                step = closedSet.get(step.parent)
        ) {
            var record = new HopRecord();
            record.index = i;
            record.time = step.cost.g;
            record.node = step.node;

            hops.add(record);

            i++;
        }

        return hops.build();
    }

    private static double distance(Node a, Node b) {
        PointValue posA = (PointValue) a.getProperty("coord");
        PointValue posB = (PointValue) b.getProperty("coord");

        final var calc = posA.getCoordinateReferenceSystem().getCalculator();

        return calc.distance(posA, posB);
    }

    private static double heuristic(Node a, Node b, double maxspeed) {
        // Maximum speed limit in UK: 70mph
        return (distance(a, b) / (maxspeed + 15) * MPH_TO_MS);
    }

    private static double crossTime(Relationship way, Node a, Node b, double maxspeed) {
        double distance = distance(a,b);
        return Math.max( // We take the longest cross time (ie lower speed)
                distance / (Double) way.getProperty("maxspeed"),
                distance / (maxspeed * MPH_TO_MS)
        );
    }

    private static long getIdOf(Node node) {
        return (long) node.getProperty("id");
    }

    private static class Cost implements Comparable<Cost> {
        public double g;
        public double h;

        public Cost(double g, double h) {
            this.g = g;
            this.h = h;
        }

        @Override
        public int compareTo(Cost b) {
            return Double.compare(g + h, b.g + b.h);
        }
    }

    private static class RouteNode implements Comparable<RouteNode> {
        public Cost cost;
        public Node node;

        public long parent;

        public RouteNode(Cost cost, Node node) {
            this(cost, node, 0L);
        }

        public RouteNode(Cost cost, Node node, long parent) {
            this.cost = cost;
            this.node = node;
            this.parent = parent;
        }

        //public void setParent(long parent) {
        //    this.parent = parent;
        //}

        @Override
        public int compareTo(RouteNode o) {
            return cost.compareTo(o.cost);
        }

        public long getId() {
            return getIdOf(this.node);
        }
    }

    public static class HopRecord {
        public long index;
        public Node node;
        public double time;

    }
}
