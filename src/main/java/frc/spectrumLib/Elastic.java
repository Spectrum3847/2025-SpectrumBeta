package frc.spectrumLib;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

public final class Elastic {
    private static final StringTopic topic =
            NetworkTableInstance.getDefault().getStringTopic("/Elastic/RobotNotifications");
    private static final StringPublisher publisher =
            topic.publish(PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));
    private static final ObjectMapper objectMapper = new ObjectMapper();

    /**
     * Sends an alert notification to the Elastic dashboard. The alert is serialized as a JSON
     * string before being published.
     *
     * @param alert the {@link ElasticNotification} object containing alert details
     */
    public static void sendAlert(ElasticNotification alert) {
        try {
            publisher.set(objectMapper.writeValueAsString(alert));
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
    }

    /**
     * Represents a notification object to be sent to the Elastic dashboard. This object holds
     * properties such as level, title, description, display time, and dimensions to control how the
     * alert is displayed on the dashboard.
     */
    public static class ElasticNotification {
        @JsonProperty("level")
        private NotificationLevel level;

        @JsonProperty("title")
        private String title;

        @JsonProperty("description")
        private String description;

        @JsonProperty("displayTime")
        private int displayTimeMillis;

        @JsonProperty("width")
        private double width;

        @JsonProperty("height")
        private double height;

        /**
         * Creates a new ElasticNotification with all default parameters. This constructor is
         * intended to be used with the chainable decorator methods
         *
         * <p>Title and description fields are empty.
         */
        public ElasticNotification() {
            this(NotificationLevel.INFO, "", "");
        }

        /**
         * Creates a new ElasticNotification with all properties specified.
         *
         * @param level the level of the notification (e.g., INFO, WARNING, ERROR)
         * @param title the title text of the notification
         * @param description the descriptive text of the notification
         * @param displayTimeMillis the time in milliseconds for which the notification is displayed
         * @param width the width of the notification display area
         * @param height the height of the notification display area, inferred if below zero
         */
        public ElasticNotification(
                NotificationLevel level,
                String title,
                String description,
                int displayTimeMillis,
                double width,
                double height) {
            this.level = level;
            this.title = title;
            this.displayTimeMillis = displayTimeMillis;
            this.description = description;
            this.height = height;
            this.width = width;
        }

        /**
         * Creates a new ElasticNotification with default display time and dimensions.
         *
         * @param level the level of the notification
         * @param title the title text of the notification
         * @param description the descriptive text of the notification
         */
        public ElasticNotification(NotificationLevel level, String title, String description) {
            this(level, title, description, 3000, 350, -1);
        }

        /**
         * Creates a new ElasticNotification with a specified display time and default dimensions.
         *
         * @param level the level of the notification
         * @param title the title text of the notification
         * @param description the descriptive text of the notification
         * @param displayTimeMillis the display time in milliseconds
         */
        public ElasticNotification(
                NotificationLevel level, String title, String description, int displayTimeMillis) {
            this(level, title, description, displayTimeMillis, 350, -1);
        }

        /**
         * Creates a new ElasticNotification with specified dimensions and default display time. If
         * the height is below zero, it is automatically inferred based on screen size.
         *
         * @param level the level of the notification
         * @param title the title text of the notification
         * @param description the descriptive text of the notification
         * @param width the width of the notification display area
         * @param height the height of the notification display area, inferred if below zero
         */
        public ElasticNotification(
                NotificationLevel level,
                String title,
                String description,
                double width,
                double height) {
            this(level, title, description, 3000, width, height);
        }

        /**
         * Updates the level of this notification
         *
         * @param level the level to set the notification to
         */
        public void setLevel(NotificationLevel level) {
            this.level = level;
        }

        /** @return the level of this notification */
        public NotificationLevel getLevel() {
            return level;
        }

        /**
         * Updates the title of this notification
         *
         * @param title the title to set the notification to
         */
        public void setTitle(String title) {
            this.title = title;
        }

        /**
         * Gets the title of this notification
         *
         * @return the title of this notification
         */
        public String getTitle() {
            return title;
        }

        /**
         * Updates the description of this notification
         *
         * @param description the description to set the notification to
         */
        public void setDescription(String description) {
            this.description = description;
        }

        public String getDescription() {
            return description;
        }

        /**
         * Updates the display time of the notification
         *
         * @param seconds the number of seconds to display the notification for
         */
        public void setDisplayTimeSeconds(double seconds) {
            setDisplayTimeMillis((int) Math.round(seconds * 1000));
        }

        /**
         * Updates the display time of the notification in milliseconds
         *
         * @param displayTimeMillis the number of milliseconds to display the notification for
         */
        public void setDisplayTimeMillis(int displayTimeMillis) {
            this.displayTimeMillis = displayTimeMillis;
        }

        /**
         * Gets the display time of the notification in milliseconds
         *
         * @return the number of milliseconds the notification is displayed for
         */
        public int getDisplayTimeMillis() {
            return displayTimeMillis;
        }

        /**
         * Updates the width of the notification
         *
         * @param width the width to set the notification to
         */
        public void setWidth(double width) {
            this.width = width;
        }

        /**
         * Gets the width of the notification
         *
         * @return the width of the notification
         */
        public double getWidth() {
            return width;
        }

        /**
         * Updates the height of the notification
         *
         * <p>If the height is set to -1, the height will be determined automatically by the
         * dashboard
         *
         * @param height the height to set the notification to
         */
        public void setHeight(double height) {
            this.height = height;
        }

        /**
         * Gets the height of the notification
         *
         * @return the height of the notification
         */
        public double getHeight() {
            return height;
        }

        /**
         * Modifies the notification's level and returns itself to allow for method chaining
         *
         * @param level the level to set the notification to
         * @return the current notification
         */
        public ElasticNotification withLevel(NotificationLevel level) {
            this.level = level;
            return this;
        }

        /**
         * Modifies the notification's title and returns itself to allow for method chaining
         *
         * @param title the title to set the notification to
         * @return the current notification
         */
        public ElasticNotification withTitle(String title) {
            setTitle(title);
            return this;
        }

        /**
         * Modifies the notification's description and returns itself to allow for method chaining
         *
         * @param description the description to set the notification to
         * @return the current notification
         */
        public ElasticNotification withDescription(String description) {
            setDescription(description);
            return this;
        }

        /**
         * Modifies the notification's display time and returns itself to allow for method chaining
         *
         * @param seconds the number of seconds to display the notification for
         * @return the current notification
         */
        public ElasticNotification withDisplaySeconds(double seconds) {
            return withDisplayMilliseconds((int) Math.round(seconds * 1000));
        }

        /**
         * Modifies the notification's display time and returns itself to allow for method chaining
         *
         * @param displayTimeMillis the number of milliseconds to display the notification for
         * @return the current notification
         */
        public ElasticNotification withDisplayMilliseconds(int displayTimeMillis) {
            setDisplayTimeMillis(displayTimeMillis);
            return this;
        }

        /**
         * Modifies the notification's width and returns itself to allow for method chaining
         *
         * @param width the width to set the notification to
         * @return the current notification
         */
        public ElasticNotification withWidth(double width) {
            setWidth(width);
            return this;
        }

        /**
         * Modifies the notification's height and returns itself to allow for method chaining
         *
         * @param height the height to set the notification to
         * @return the current notification
         */
        public ElasticNotification withHeight(double height) {
            setHeight(height);
            return this;
        }

        /**
         * Modifies the notification's height and returns itself to allow for method chaining
         *
         * <p>This will set the height to -1 to have it automatically determined by the dashboard
         *
         * @return the current notification
         */
        public ElasticNotification withAutomaticHeight() {
            setHeight(-1);
            return this;
        }

        /**
         * Modifies the notification to disable the auto dismiss behavior
         *
         * <p>This sets the display time to 0 milliseconds
         *
         * <p>The auto dismiss behavior can be re-enabled by setting the display time to a number
         * greater than 0
         *
         * @return the current notification
         */
        public ElasticNotification withNoAutoDismiss() {
            setDisplayTimeMillis(0);
            return this;
        }

        /**
         * Represents the possible levels of notifications for the Elastic dashboard. These levels
         * are used to indicate the severity or type of notification.
         */
        public enum NotificationLevel {
            /** Informational Message */
            INFO,
            /** Warning message */
            WARNING,
            /** Error message */
            ERROR
        }
    }
}
