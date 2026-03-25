package robotutils;

import static org.junit.jupiter.api.Assertions.assertEquals;

class TestTest {

    @org.junit.jupiter.api.Test
    void getTheValueReturnsFive() {
        Test test = new Test();

        assertEquals(5, test.getTheValue());
    }
}