package robotutils.perrobotconfig;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import org.junit.jupiter.api.Test;
import robotutils.interfaces.MacKey;


class TestPerRobotConfig {

    interface TestConfigInterface {}

    class TestConfig1 implements TestConfigInterface {
        public final int m_value = 5;

        public TestConfig1() {
        }
    }

    class TestConfig2 implements TestConfigInterface {
        public final int m_value = 42;

        public TestConfig2() {
        }
    }

    // ---------------------------------------------------------------------------
    // Minimal valid inputs reused across validation tests
    // ---------------------------------------------------------------------------
    private final MacKey m_macKeyA = new MacKey(0x01, 0x02);
    private final MacKey m_macKeyB = new MacKey(0x03, 0x04);
    private final String m_robotNameA = "RobotComp";
    private final String m_robotNameB = "RobotTesting";

    private final String m_configNameA = "ConfigA";
    private final TestConfigInterface m_configObjA = new TestConfig1();

    private final String m_configNameB = "ConfigB";
    private final TestConfigInterface m_configObjB = new TestConfig2();

    private final Map<MacKey, String> m_validMacDict = Map.of(
        m_macKeyA, m_robotNameA,
        m_macKeyB, m_robotNameB);
    private final Map<String, String> m_validNameDict = Map.of(
        m_robotNameA, m_configNameA,
        m_robotNameB, m_configNameB);
    private final Map<String, TestConfigInterface> m_validConfigDict = Map.of(
        m_configNameA, m_configObjA,
        m_configNameB, m_configObjB);

    /** Passing a null macToRobotNameDict throws IllegalArgumentException. */
    @Test
    void constructor_nullMacDict_throwsIllegalArgument() {
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                null,
                m_validNameDict,
                m_validConfigDict,
                m_configNameA,
                m_configNameA));
        assertEquals("macToRobotNameDict must contain at least one entry", ex.getMessage());
    }

    /** Passing an empty macToRobotNameDict throws IllegalArgumentException. */
    @Test
    void constructor_emptyMacDict_throwsIllegalArgument() {
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                Map.of(),
                m_validNameDict,
                m_validConfigDict,
                m_configNameA,
                m_configNameA));
        assertEquals("macToRobotNameDict must contain at least one entry", ex.getMessage());
    }

    /**
     * A robot name in the MAC dict that has no entry in the name dict throws
     * IllegalArgumentException.
     */
    @Test
    void constructor_robotNameMissingFromNameDict_throwsIllegalArgument() {
        // Name dict has entries but none matching the robot names in VALID_MAC_DICT
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                m_validMacDict,
                Map.of("UnrelatedRobot", m_configNameA),
                m_validConfigDict,
                m_configNameA,
                m_configNameA));
        assertTrue(ex.getMessage().startsWith("Missing robot->config mapping for robot name: "));
    }

    /**
     * A config name in the name dict that has no entry in the config dict throws
     * IllegalArgumentException.
     */
    @Test
    void constructor_configNameMissingFromConfigDict_throwsIllegalArgument() {
    }

    /**
     * A defaultConfigName that is not present in the config dict throws
     * IllegalArgumentException.
     */
    @Test
    void constructor_invalidDefaultConfigName_throwsIllegalArgument() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * A simulationConfigName that is not present in the config dict
     * throws IllegalArgumentException.
     */
    @Test
    void constructor_invalidSimulationConfigName_throwsIllegalArgument() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * When running in simulation, getBotConfigName always returns the
     * simulationConfigName.
     */
    @Test
    void constructor_inSimulation_returnsSimulationConfig() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** When running in simulation, getRobotName returns "Simulation". */
    @Test
    void constructor_inSimulation_robotNameIsSimulation() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * When the testMacKey matches a known MAC entry, getBotConfigName
     * returns that robot's config.
     * */
    @Test
    void testMacKey_matchingEntry_returnsMatchedRobotConfig() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * When the testMacKey matches a known MAC entry, getRobotName returns that
     * robot's name.
     */
    @Test
    void testMacKey_matchingEntry_returnsCorrectRobotName() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * When the testMacKey does not match any known MAC, getBotConfigName
     * returns the defaultConfigName.
     */
    @Test
    void testMacKey_noMatchingEntry_returnsDefaultConfig() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * When the testMacKey does not match any known MAC, getRobotName
     * returns "Unknown Robot".
     */
    @Test
    void testMacKey_noMatchingEntry_robotNameIsUnknownRobot() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * getBotConfig returns the config object associated with the
     * selected config name.
     */
    @Test
    void getBotConfig_returnsSelectedConfigObject() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** getBotConfigName returns the string name of the selected config. */
    @Test
    void getBotConfigName_returnsSelectedConfigName() {
        throw new UnsupportedOperationException("Not implemented");
    }
}
