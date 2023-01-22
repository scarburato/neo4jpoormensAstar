package londonSafeTravel;

import org.neo4j.graphdb.*;
import org.neo4j.logging.Log;
import org.neo4j.procedure.*;
import org.neo4j.values.storable.PointValue;

import java.util.*;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

public class RoutingAStart {
    static final double MPH_TO_MS = 0.44704;
    static final Label POINT = Label.label("Point");
    static final RelationshipType CONNECTS = RelationshipType.withName("CONNECTS");
    static final RelationshipType IS_DISRUPTED = RelationshipType.withName("IS_DISRUPTED");

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

    static final HashMap<String, Double> disruptionWeights = new HashMap<>(){{
        // [ 'Minimal', 'Moderate', 'Serious', 'Severe' ]
        put("Minimal", 1.1);
        put("Moderate", 2.0);
        put("Serious", 3.333);
        put("Severe", 6.0);
    }};

    @Context
    public Log log;

    @Procedure(value = "londonSafeTravel.route", mode = Mode.READ)
    @Description("Finds the optimal path between two POINTS")
    public Stream<HopRecord> route(
            @Name("start") Node start,
            @Name("end") Node end,
            @Name("crossTimeField") String crossTimeField,
            @Name("considerDisruptions") boolean considerDisruptions,
            @Name("maxSpeed") double maxspeed
    ) {
        return route2(start, end, crossTimeField, considerDisruptions, maxspeed, 1.0);
    }
    @Procedure(value = "londonSafeTravel.route.anytime", mode = Mode.READ)
    @Description("Finds the sub-optimal path between two POINTS")
    public Stream<HopRecord> route2(
            @Name("start") Node start,
            @Name("end") Node end,
            @Name("crossTimeField") String crossTimeField,
            @Name("considerDisruptions") boolean considerDisruptions,
            @Name("maxSpeed") double maxspeed,
            @Name("w") double weight
    ) {
        if(!start.hasLabel(POINT))
            throw new IllegalArgumentException("`start' does not have `Point' as a label");

        if(!end.hasLabel(POINT))
            throw new IllegalArgumentException("`end' does not have `Point' as a label");

        if(weight < 1.0 || weight == Double.POSITIVE_INFINITY)
            throw new IllegalArgumentException("`weight' must be in [1, +Infinity)");

        TreeSet<RouteNode> openSetHeap = new TreeSet<>();
        HashMap<Long, RouteNode> openSet = new HashMap<>(0xfff, 0.666f);
        HashMap<Long, RouteNode> closedSet = new HashMap<>(0xfeee, 0.666f);

        var startNode = new RouteNode(
                new Cost(0, weight * heuristic(start, end, maxspeed)), start);
        openSetHeap.add(startNode);
        openSet.put(startNode.getId(), startNode);

        log.info("Starting route from " + getIdOf(start) + "\tto " + getIdOf(end));

        RouteNode current = null;
        while(! openSet.isEmpty()) {
            assert (openSet.size() == openSetHeap.size());

            // Get current node
            current = openSetHeap.pollFirst(); // O(log n)
            openSet.remove(current.getId()); // ~O(1)

            // Move from open to closed
            closedSet.put(current.getId(), current); // ~O(1)

            // Found the solution HALT!
            if(current.getId() == getIdOf(end))
                break;

            var hops = current.node.getRelationships(Direction.OUTGOING, CONNECTS); // O(1), I hope?

            for(var way : hops) {
                // NO ENTRY IN HERE!
                final double crossTimeStored = (Double) way.getProperty(crossTimeField);
                if(crossTimeStored == Double.POSITIVE_INFINITY)
                    continue;

                // Successor info
                Node successor = way.getEndNode();
                final long successorId = getIdOf(successor);

                // cost for this edge
                double crossTime = crossTime(way, current.node, successor, maxspeed);

                // Look for disruption
                if(considerDisruptions)
                    crossTime = considerDisruptions(successor, crossTime);

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

                boolean toInsert = true;

                // if in open list and g' < g
                RouteNode routeHop = openSet.get(successorId); // ~O(1)
                if(routeHop != null) {
                    if(travelTime < routeHop.cost.g) {
                        openSetHeap.remove(routeHop); // O(log n)
                        openSet.remove(successorId); // ~O(1)
                    } else
                        toInsert = false;
                }

                // if in closed and g' < g
                if(routeHop == null) {
                    routeHop = closedSet.get(successorId);
                    if (routeHop != null)
                        if(travelTime < routeHop.cost.g) {
                            closedSet.remove(successorId); // O(1)
                            //toInsert = true;
                        } else
                            toInsert = false;
                }

                if(! toInsert)
                    continue;

                if(routeHop == null)
                    routeHop = new RouteNode();

                // We found a better path OR We first visited this node!
                routeHop.cost.g = travelTime;
                routeHop.cost.h = weight * heuristic(successor, end, maxspeed);
                routeHop.node = successor;
                routeHop.parent = current.getId();

                openSet.put(successorId, routeHop);
                openSetHeap.add(routeHop);
            }
        }

        log.info(
                "A* terminated in " + current.getId() +
                "\topenSet size: " + openSet.size() + "\tclosedSet size: " + closedSet.size()
        );

        if(current.getId() != getIdOf(end))
            return Stream.<HopRecord>builder().build();

        // Java non-sense
        final RouteNode finalCurrent = current;
        return StreamSupport.stream(Spliterators.spliteratorUnknownSize(new Iterator<>() {
            long i = 0;
            RouteNode step = finalCurrent;

            @Override
            public boolean hasNext() {
                return step.getId() != getIdOf(start);
            }

            @Override
            public HopRecord next() {
                var record = new HopRecord();
                record.index = i;
                record.time = step.cost.g;
                record.node = step.node;

                step = closedSet.get(step.parent);
                i++;

                return record;
            }
        }, Spliterator.IMMUTABLE), false);
    }

    private double considerDisruptions(Node successor, double crossTime) {
        var disruptionEdges = successor.getRelationships(Direction.OUTGOING, IS_DISRUPTED);
        for (var disruptionEdge : disruptionEdges) {
            Node disruption = disruptionEdge.getEndNode();
            if (!disruption.hasProperty("severity")) {
                log.warn("Disruption " + disruption.getElementId() + " has no severity!!");
                continue;
            }

            Double dw = disruptionWeights.get((String) disruption.getProperty("severity"));
            if (dw == null) {
                log.warn("Uknown severity " + disruption.getProperty("severity"));
                continue;
            }

            if ((Boolean) disruption.getProperty("closed", false)) {
                log.debug(
                        "Closed road at " + disruption.getElementId() +
                        "\tPoint " + successor.getElementId()
                );

                return crossTime * 25;
            }

            crossTime *= dw;
        }

        return crossTime;
    }

    private static double distance(Node a, Node b) {
        PointValue posA = (PointValue) a.getProperty("coord");
        PointValue posB = (PointValue) b.getProperty("coord");

        final var calc = posA.getCoordinateReferenceSystem().getCalculator();

        return calc.distance(posA, posB);
    }

    private static double heuristic(Node a, Node b, double maxspeed) {
        // Maximum speed limit in UK: 70mph
        return (distance(a, b) / (maxspeed + 5) * MPH_TO_MS);
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

        public Cost() {}

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

        public RouteNode() {
            this.cost = new Cost();
        }

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
