# include("jl")

# using Main.LibRealSense

# import Main.LibRealSense: *

# librealsense2 = "/home/user/src/librealsense/bb/librealsense2.so"


using Images, ImageView, GtkReactive

using RealSense

function checkerror(err)
    iserr = err[] != C_NULL
    if iserr
        funcname = unsafe_string(rs2_get_failed_function(err[]))
        funcargs = unsafe_string(rs2_get_failed_args(err[]))
        message = unsafe_string(rs2_get_error_message(err[]))
        @error "rs_error was raised when calling $funcname($funcargs):\n    $message"
    end
    return iserr
end

function deviceinfo(dev)
    err = Ref{Ptr{rs2_error}}(0)
    device_info = rs2_get_device_info(dev, RS2_CAMERA_INFO_NAME, err)
    checkerror(err)
    @info "This device is an $(unsafe_string(device_info))"
    device_info = rs2_get_device_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, err)
    checkerror(err)
    @info "  Serial number: $(unsafe_string(device_info))"
    device_info = rs2_get_device_info(dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, err)
    checkerror(err)
    @info "  Firmware version: $(unsafe_string(device_info))"
    device_info = rs2_get_device_info(dev, RS2_CAMERA_INFO_PHYSICAL_PORT, err)
    checkerror(err)
    @info "  Physical port: $(unsafe_string(device_info))"
    device_info = rs2_get_device_info(dev, RS2_CAMERA_INFO_ADVANCED_MODE, err)
    checkerror(err)
    @info "  Advanced mode: $(unsafe_string(device_info))"
    device_info = rs2_get_device_info(dev, RS2_CAMERA_INFO_PRODUCT_ID, err)
    checkerror(err)
    @info "  Product ID: $(unsafe_string(device_info))"
end



err = Ref{Ptr{rs2_error}}(0)

ctx = rs2_create_context(RS2_API_VERSION, err)
checkerror(err)

device_list = rs2_query_devices(ctx, err)
checkerror(err)

dev_count = rs2_get_device_count(device_list, err)
checkerror(err)

@info "There are $dev_count connected RealSense devices."


# rs2_create_sensor(list, index, error)

dev = rs2_create_device(device_list, 0, err)
checkerror(err)
# deviceinfo(dev)

sensor_list = rs2_query_sensors(dev, err)
checkerror(err)

num_of_sensors = rs2_get_sensors_count(sensor_list, err)
checkerror(err)
@info "There are $num_of_sensors sensors."

sensor = rs2_create_sensor(sensor_list, 0, err)
checkerror(err)
rs2_set_option(sensor, RS2_OPTION_LASER_POWER, 0f0, err)
checkerror(err)
cfg = rs2_create_config(err)
checkerror(err)
rs2_config_enable_stream(cfg, RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 6, err)
checkerror(err)
rs2_config_enable_stream(cfg, RS2_STREAM_INFRARED, 2, 1280, 720, RS2_FORMAT_Y8, 6, err)
checkerror(err)

myshape = (1280,720)

pipe = rs2_create_pipeline(ctx, err)
# rs2_pipeline_start(pipe, err)
rs2_pipeline_start_with_config(pipe, cfg, err)
checkerror(err)


function getframes()
    frames = rs2_pipeline_wait_for_frames(pipe, RS2_DEFAULT_TIMEOUT, err)
    checkerror(err)

    num_of_frames = rs2_embedded_frames_count(frames, err)
    checkerror(err)
    
    theframes = map(0:(num_of_frames-1)) do i
        frame = rs2_extract_frame(frames, i, err)
        checkerror(err)

        # rgb_frame_data = Ptr{UInt8}(rs2_get_frame_data(frame, err))
        rgb_frame_data_ptr = Ptr{UInt8}(rs2_get_frame_data(frame, err))
        checkerror(err)
        # rgb_frame_data = unsafe_wrap(Vector{UInt8}, rgb_frame_data_ptr, prod(myshape))
        # append!(mydata, rgb_frame_data[:])

        frame_number = rs2_get_frame_number(frame, err)
        checkerror(err)

        frame_timestamp = rs2_get_frame_timestamp(frame, err)
        checkerror(err)

        frame_timestamp_domain = rs2_get_frame_timestamp_domain(frame, err)
        checkerror(err)
        
        frame_timestamp_domain_str = rs2_timestamp_domain_to_string(frame_timestamp_domain)

        frame_metadata_time_of_arrival = rs2_get_frame_metadata(frame, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, err)
        checkerror(err)

        # println("New frame arrived.")
        # print("First 10 bytes: ")
        # print(rgb_frame_data[1:11])
        # println("\nFrame No: ", frame_number)
        # println("Timestamp: ", frame_timestamp)
        # println("Timestamp domain: ", frame_timestamp_domain_str)
        # println("Time of arrival: \n", frame_metadata_time_of_arrival)

        # frm = reinterpret(Gray{N0f8}, permutedims(unsafe_wrap(Matrix{UInt8}, rgb_frame_data_ptr, myshape)))
        frm = reinterpret(Gray{N0f8}, unsafe_wrap(Matrix{UInt8}, rgb_frame_data_ptr, myshape))

        rs2_release_frame(frame)
        frm
    end
    out = [Float32.(theframes[1]');Float32.(theframes[2]')]
    
    rs2_release_frame(frames)
    # theframes
    out
end

struct BGmodel
    μ
    ρ
    ϵfg
end

function mahalanobispoisson(x, S)
    x * (x / S - 2) + S
end

FULL_WELL_CAPACITY = 16000/255.0
THRESHOLD = 3.0
function detect(bgmodel, img)
    jj,kk = size(img)
    [mahalanobispoisson(img[j,k], bgmodel.μ[j,k]) * FULL_WELL_CAPACITY > THRESHOLD^2 for j in 1:jj, k in 1:kk]
end

function update(bgmodel, img)
    bgmodel.μ .= bgmodel.μ + bgmodel.ρ * (img - bgmodel.μ)
end


# mydata = [getframes() for it in 1:128]

# c1 = imshow(mydata[end][1])["gui"]["canvas"]
# s1 = Signal(mydata[end][1])
# imshow(c1, s1)
# c2 = imshow(mydata[end][2])["gui"]["canvas"]
# s2 = Signal(mydata[end][2])
# imshow(c2, s2)

# i1,i2 = getframes()
# push!(s1, i1)
# push!(s2, i2)

# i1,i2 = getframes()
# i12 = [i1 i2]
i12 = reshape(getframes(), :, 1280)
b1 = BGmodel(i12, 0.01, 0.1)
fg = detect(b1, i12)
ii = [i12 b1.μ fg]
c1 = imshow(ii)["gui"]["canvas"]
s1 = Signal(ii)
imshow(c1, s1)

# for x in 1:120
@async while true
    i12 = reshape(getframes(), :, 1280)
    update(b1, i12)
    fg = detect(b1, i12)
    ii = [i12 b1.μ fg]
    push!(s1, ii)
    # push!(s1, i12)
    # push!(s1, b1.μ)
    sleep(0.01)
end

# i1,i2 = getframes(); push!(s1, [i1 i2])

# push!(imgsig, newimg)





