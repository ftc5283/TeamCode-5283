package org.firstinspires.ftc.teamcode.pipeline;

import androidx.annotation.NonNull;

import org.apache.commons.math3.linear.MatrixDimensionMismatchException;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
/** @noinspection StringEquality*/
public class TelemetryPipeline{
    public final Telemetry telemetry;
    public final ArrayList<String> keys = new ArrayList<>();
    public final ArrayList<String> data = new ArrayList<>();

    public TelemetryPipeline(@NonNull Telemetry telemetry) {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);
        this.telemetry = telemetry;
    }

    /**
     * Adds a data point that is removed after {@link TelemetryPipeline#refresh()}.Recommended when
     * NOT running code in parallel.
     *
     * @param key A description of what value represents. The left half of the data point displayed
     *            on the driver station. Example: {@literal "Step"}.
     * @param value The value displayed. The right half of what is displayed on the driver station.
     *              Example: {@literal "Park in Observation Zone"}.
     *
     * @see TelemetryPipeline#addDataPoint(String, Object)
     * @see TelemetryPipeline#addDataPoint(String, double)
     * @see TelemetryPipeline#addDataPoint(String, boolean)
     * @see TelemetryPipeline#addDataPointPerpetual(String, String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, Object)
     */
    public void addDataPoint(@NonNull String key, @NonNull  String value) {
        telemetry.addData(key, value);
    }


    /**
     * Adds a data point that is removed after {@link TelemetryPipeline#refresh()}.Recommended when
     * NOT running code in parallel.
     *
     * @param key A description of what value represents. The left half of the data point displayed
     *            on the driver station. Example: {@literal "Step"}.
     * @param value The value displayed. The right half of what is displayed on the driver station.
     *              Example: {@literal "Park in Observation Zone"}.
     *
     * @see TelemetryPipeline#addDataPoint(String, Object)
     * @see TelemetryPipeline#addDataPoint(String, double)
     * @see TelemetryPipeline#addDataPoint(String, String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, Object)
     */
    public void addDataPoint(@NonNull String key, boolean value) {
        telemetry.addData(key, value);
    }

    /**
     * Adds a data point that is removed after {@link TelemetryPipeline#refresh()}.Recommended when
     * NOT running code in parallel.
     *
     * @param key A description of what value represents. The left half of the data point displayed
     *            on the driver station. Example: {@literal "Step"}.
     * @param value The value displayed. The right half of what is displayed on the driver station.
     *              Example: {@literal "Park in Observation Zone"}.
     *
     * @see TelemetryPipeline#addDataPoint(String, Object)
     * @see TelemetryPipeline#addDataPoint(String, boolean)
     * @see TelemetryPipeline#addDataPoint(String, String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, Object)
     */
    public void addDataPoint(@NonNull String key, double value) {
        telemetry.addData(key, value);
    }

    /**
     * Adds a data point that is removed after {@link TelemetryPipeline#refresh()}. Recommended when
     * NOT running code in parallel.
     *
     * @param key A description of what value represents. The left half of the data point displayed
     *            on the driver station. Example: {@literal "Arm Position"}.
     * @param value The value displayed. The right half of what is displayed on the driver station.
     *              Example: {@code 3}. Can be a primitive.
     *
     * @see TelemetryPipeline#addDataPoint(String, String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, Object)
     */
    public void addDataPoint(@NonNull String key, @NonNull  Object value) {
        telemetry.addData(key, value);
    }

    /**
     * Adds a data point is not removed after {@link TelemetryPipeline#refresh()}. If a perpetual
     * data point with the same key already exists, its value is set the provided value instead of
     * adding a new data point. Recommended when running code in parallel.
     *
     * @param key A description of what value represents. The left half of the data point displayed
     *            on the driver station. Example: {@literal "Step"}.
     * @param value The value displayed. The right half of what is displayed on the driver station.
     *              Example: {@literal "Park in Observation Zone"}.
     *
     * @see TelemetryPipeline#removeDataPoint(String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, String)
     * @see TelemetryPipeline#addDataPoint(String, String)
     * @see TelemetryPipeline#addDataPoint(String, Object)
     */
    public boolean addDataPointPerpetual(@NonNull String key, @NonNull String value) {
        key = key.intern();
        for(int i = 0; i<keys.size(); i++){
            if(keys.get(i) == key) {
                data.set(i, value);
                return false;
            }
        }
        keys.add(key);
        data.add(value);
        return true;
    }

    /**
     * Adds a data point is not removed after {@link TelemetryPipeline#refresh()}. If a data point
     * with the same key already exists, its value is set the provided value instead of adding a new
     * data point. Recommended when running code in parallel.
     *
     * @param key A description of what value represents. The left half of the data point displayed
     *            on the driver station. Example: {@literal "Arm Position"}.
     * @param value The value displayed. The right half of what is displayed on the driver station.
     *              Example: {@code 3}. Can be a primitive.
     *
     * @return {@code true} if the key is new. <br>
     *         {@code false} if the key already existed.
     *
     * @see TelemetryPipeline#removeDataPoint(String)
     * @see TelemetryPipeline#addDataPointPerpetual(String, Object)
     * @see TelemetryPipeline#addDataPoint(String, String)
     * @see TelemetryPipeline#addDataPoint(String, Object)
     */
    public boolean addDataPointPerpetual(@NonNull String key, @NonNull Object value) {
        return addDataPointPerpetual(key, value.toString());
    }

    /**
     * Adds a header that is removed after {@link TelemetryPipeline#refresh()}. Recommended when NOT
     * running code in parallel.
     *
     * @param header The text of the header.
     *
     * @see TelemetryPipeline#addHeaderPerpetual(String)
     */
    public void addHeader(@NonNull String header) {
        telemetry.addLine(header);
    }

    /**
     * Adds a header that is NOT removed after {@link TelemetryPipeline#refresh()}. Recommended when
     * running code in parallel.
     *
     * @param header The text of the header.
     *
     * @return {@code true} if the key is new. <br>
     *         {@code false} if the key already existed.
     *
     * @see TelemetryPipeline#addHeader(String)
     * @see TelemetryPipeline#removeHeader(String)
     */
    public boolean addHeaderPerpetual(@NonNull String header) {
        header = header.intern();
        for(int i = 0; i<keys.size(); i++){
            if(keys.get(i) == header) {
                data.set(i, null);
                return false;
            }
        }
        keys.add(header);
        data.add(null);
        return true;
    }

    /**
     * Adds a blank line that is removed after {@link TelemetryPipeline#refresh()}. Recommended when
     * NOT running code in parallel.
     *
     * @see TelemetryPipeline#addLinePerpetual()
     */
    public void addLine() {
        telemetry.addLine();
    }

    /**
     * Adds a blank line that is not removed after {@link TelemetryPipeline#refresh()}. Recommended
     * when running code in parallel.
     *
     * @see TelemetryPipeline#removeLine(int)
     * @see TelemetryPipeline#addLine()
     */
    public void addLinePerpetual() {
        keys.add(null);
        data.add(null);
    }

    /**
     * Attempts to remove the first perpetual data point that matches the specified key.
     *
     * @param key The key of the perpetual data point to remove.
     *
     * @return {@code true} if the data point was removed. <br>
     *         {@code false} if the data point does not exist.
     *
     * @see TelemetryPipeline#addDataPoint(String, String)
     * @see TelemetryPipeline#addDataPoint(String, Object)
     */
    public boolean removeDataPoint(@NonNull String key){
        key = key.intern();
        for(int i = 0; i<keys.size(); i++) {
            String compare = keys.get(i);
            if (compare != null && compare == key && data.get(i) != null) {
                keys.remove(i);
                data.remove(i);
                return true;
            }
        }
        return false;
    }

    /**
     * Attempts to remove the first perpetual header that matches the specified text.
     *
     * @param text The text of the perpetual header to remove.
     *
     * @return {@code true} if the header was removed. <br>
     *         {@code false} if the header does not exist.
     *
     * @see TelemetryPipeline#addHeaderPerpetual(String)
     */
    public boolean removeHeader(@NonNull String text){
        text = text.intern();
        for(int i = 0; i<keys.size(); i++) {
            String compare = keys.get(i);
            if (compare != null && compare == text && data.get(i) == null) {
                keys.remove(i);
                data.remove(i);
                return true;
            }
        }
        return false;
    }

    /**
     * Attempts to remove the specified perpetual empty line.
     *
     * @param lineNumber How many empty lines to skip before removing an empty line
     *
     * @return {@code true} if the empty line was removed. <br>
     *         {@code false} if the empty line does not exist.
     *
     * @see TelemetryPipeline#addLinePerpetual()
     */
    public boolean removeLine(int lineNumber){
        if(lineNumber < 0)
            throw new IllegalArgumentException("lineNumber must be non-negative");
        for(int i = 0; i<keys.size(); i++) {
            if (keys.get(i) == null && lineNumber-- == 0) {
                keys.remove(i);
                data.remove(i);
                return true;
            }
        }
        return false;
    }

    /**
     * Displays and & removes all non-perpetual data, headers and empty-lines added since last called.
     * Then displays without removal all perpetual data, headers and empty-lines.
     * Runs {@link TelemetryPipeline#audit()} upon completion.
     */
    public void refresh() {
        for(int i = 0; i<keys.size(); i++){
            String key = keys.get(i);
            String data = this.data.get(i);
            if (key == null)
                addLine();
            else if(data == null)
                addHeader(key);
            else
                addDataPoint(key, data);
        }
        telemetry.update();
        audit();
    }

    /**
     * Throws an exception if the state of this Telemetry Pipeline is illegal.
     */
    public void audit(){
        if(keys.size() != data.size())
            throw new RuntimeException(
                    "Size of keys does not match size of data.\n"+
                            "keys.size() = "+keys.size()+"\n"+
                            "data.size() = "+data.size());

        for(int i = 0; i<keys.size(); i++)
            if(keys.get(i) == null && data.get(i) != null)
                throw new NullPointerException(
                        "Key at position "+i+" is null, but the data at that position is not null.\n"+
                                "data.get("+i+") = "+"\""+data.get(i)+"\"");
    }
}

