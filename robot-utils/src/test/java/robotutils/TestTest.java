package robotutils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TestTest {

    @Test
    public void getTheValueReturnsFive() {
        robotutils.Test test = new robotutils.Test();

        assertEquals(5, test.getTheValue());
    }
}