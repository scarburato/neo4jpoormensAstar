import londonSafeTravel.RoutingAStart;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.neo4j.driver.GraphDatabase;
import org.neo4j.harness.Neo4j;
import org.neo4j.harness.Neo4jBuilders;

import java.io.*;

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
public class TestAStart {
    private Neo4j embeddedDatabaseServer;

    @BeforeAll
    void initializeNeo4j() throws IOException {

        var sw = new StringWriter();
        try (var in = new BufferedReader(
                new InputStreamReader(new FileInputStream("schema.cypher")))) {
            in.transferTo(sw);
            sw.flush();
        }

        var schifo = Neo4jBuilders.newInProcessBuilder().withProcedure(RoutingAStart.class);

        for(var query : sw.toString().split(";"))
            schifo.withFixture(query);

        this.embeddedDatabaseServer = schifo.build();
    }

    @AfterAll
    void closeNeo4j() {
        this.embeddedDatabaseServer.close();
    }

    @Test
    void testRoute() {

        try(
                var driver = GraphDatabase.driver(embeddedDatabaseServer.boltURI());
                var session = driver.session()
        ) {
            // language=cypher
            session.run("""
MATCH (s:Point{id:99912})
MATCH (e:Point{id:1150824164})
CALL londonSafeTravel.route(s, e, "crossTimeFoot", 70.0)
YIELD index, node, time
RETURN index, node, time
ORDER BY index DESCENDING
""");

            //result.forEachRemaining(record -> {
            //    record.values().forEach(System.out::println);
            //});
        }
    }
}
