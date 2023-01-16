package londonSafeTravel;

import org.neo4j.procedure.Description;
import org.neo4j.procedure.Mode;
import org.neo4j.procedure.Procedure;

import java.util.stream.Stream;

public class CreateNode {
    @Procedure(value = "test.create", mode = Mode.READ)
    @Description("Finds the optimal path between two POINTS")
    public Stream<Record> create() {
        Record record = new Record();
        record.x = 32L;

        Stream.Builder<Record> b = Stream.builder();
        b.add(record);
        return b.build();
    }

    public static class Record {
        public Long x;
    }
}
