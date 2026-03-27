package robotutils.perrobotconfig;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Map;
import org.junit.jupiter.api.Test;
import robotutils.interfaces.MacKey;


class TestPerRobotConfig {

    interface TestConfigInterface {}

    static class TestConfig implements TestConfigInterface {
        public int value;

        public TestConfig(int value) {
            this.value = value;
        }
    }

    // ---------------------------------------------------------------------------
    // Minimal valid inputs reused across validation tests
    // ---------------------------------------------------------------------------
    private static final MacKey MACKEY_A = new MacKey(0x01, 0x02);
    private static final MacKey MACKEY_B = new MacKey(0x03, 0x04);
    private static final String ROBOT_NAME_A = "RobotComp";
    private static final String ROBOT_NAME_B = "RobotTesting";

    private static final String CONFIG_NAME_A = "ConfigA";
    private static final int CONFIG_VALUE_A = 5;
    private static final TestConfig CONFIG_OBJ_A = new TestConfig(CONFIG_VALUE_A);

    private static final String CONFIG_NAME_B = "ConfigB";
    private static final int CONFIG_VALUE_B = 10;
    private static final TestConfig CONFIG_OBJ_B = new TestConfig(CONFIG_VALUE_B);

    private static final Map<MacKey, String> VALID_MAC_DICT = Map.of(
        MACKEY_A, ROBOT_NAME_A,
        MACKEY_B, ROBOT_NAME_B);
    private static final Map<String, String> VALID_NAME_DICT = Map.of(
        ROBOT_NAME_A, CONFIG_NAME_A,
        ROBOT_NAME_B, CONFIG_NAME_B);
    private static final Map<String, TestConfig> VALID_CONFIG_DICT = Map.of(
        CONFIG_NAME_A, CONFIG_OBJ_A,
        CONFIG_NAME_B, CONFIG_OBJ_B);

    /** Passing a null macToRobotNameDict throws IllegalArgumentException. */
    @Test
    void constructor_nullMacDict_throwsIllegalArgument() {
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfig>(
                null,
                VALID_NAME_DICT,
                VALID_CONFIG_DICT,
                CONFIG_NAME_A,
                CONFIG_NAME_A));
        assertEquals("macToRobotNameDict must contain at least one entry", ex.getMessage());
    }

    /** Passing an empty macToRobotNameDict throws IllegalArgumentException. */
    @Test
    void constructor_emptyMacDict_throwsIllegalArgument() {
    }

    /** A robot name in the MAC dict that has no entry in the name dict throws IllegalArgumentException. */
    @Test
    void constructor_robotNameMissingFromNameDict_throwsIllegalArgument() {
    }

    /** A config name in the name dict that has no entry in the config dict throws IllegalArgumentException. */
    @Test
    void constructor_configNameMissingFromConfigDict_throwsIllegalArgument() {
    }

    /** A defaultConfigName that is not present in the config dict throws IllegalArgumentException. */
    @Test
    void constructor_invalidDefaultConfigName_throwsIllegalArgument() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** A simulationConfigName that is not present in the config dict throws IllegalArgumentException. */
    @Test
    void constructor_invalidSimulationConfigName_throwsIllegalArgument() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** When running in simulation, getBotConfigName always returns the simulationConfigName. */
    @Test
    void constructor_inSimulation_returnsSimulationConfig() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** When running in simulation, getRobotName returns "Simulation". */
    @Test
    void constructor_inSimulation_robotNameIsSimulation() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** When the testMacKey matches a known MAC entry, getBotConfigName returns that robot's config. */
    @Test
    void testMacKey_matchingEntry_returnsMatchedRobotConfig() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** When the testMacKey matches a known MAC entry, getRobotName returns that robot's name. */
    @Test
    void testMacKey_matchingEntry_returnsCorrectRobotName() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** When the testMacKey does not match any known MAC, getBotConfigName returns the defaultConfigName. */
    @Test
    void testMacKey_noMatchingEntry_returnsDefaultConfig() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** When the testMacKey does not match any known MAC, getRobotName returns "Unknown Robot". */
    @Test
    void testMacKey_noMatchingEntry_robotNameIsUnknownRobot() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /** getBotConfig returns the config object associated with the selected config name. */
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
