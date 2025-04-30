from bps_oculus import oculus, helpers
import numpy as np
from typing import Union, Tuple
from dataclasses import dataclass
import cv2


def pack_oculus_message(item_message: Union[
    oculus.OculusSimplePingResult, oculus.OculusSimplePingResult2], item_data: oculus.OculusPolarImage) -> bytes:
    raise NotImplementedError("This function doesn't work as expected. Come back later.")
    message_bytes = item_message.pack()
    # TODO: Right now the bytes between the SimplePingResult header and the image data should contain the bearing bytes and then something else.
    bearing_bytes = (item_data.bearing_table * 180.0 / np.pi * 100).astype(np.int16).tobytes().ljust(item_message.imageOffset, b'\x00')
    image_bytes = item_data.polar_image.tobytes()
    return message_bytes + bearing_bytes + image_bytes


def filter_gain_result(item_message: Union[
    oculus.OculusSimplePingResult, oculus.OculusSimplePingResult2]) -> Union[
    oculus.OculusSimplePingResult, oculus.OculusSimplePingResult2]:
    flags = helpers.uint8_to_bits_little_endian(item_message.fireMessage.flags)
    if not flags[2]:
        return item_message
    item_message.imageSize = oculus.enum_DataSizeType_to_size(
        item_message.dataSize) * item_message.nBeams * item_message.nRanges  # the new payloadSize does not have offset
    # The flags must be updated to indicate that gain is not included
    flags[2] = 0
    item_message.fireMessage.flags = helpers.bits_to_uint8_little_endian(flags)
    # The total messageSize must be updated
    item_message.messageSize = item_message.imageSize + item_message.imageOffset
    item_message.fireMessage.head.payloadSize = item_message.imageSize + item_message.imageOffset - oculus.OculusMessageHeader.size()
    return item_message


def parse_polar_image(item_payload: bytes, item_message: Union[
    oculus.OculusSimplePingResult, oculus.OculusSimplePingResult2]) -> oculus.OculusPolarImage:
    head = item_message.fireMessage.head

    assert head.payloadSize + oculus.OculusMessageHeader.size() == item_message.imageOffset + item_message.imageSize

    size = item_message.size()

    # If gain is sent, then each range row has 4 bytes extra at the start of the row, need to check flags for gain presence / absence
    flags = helpers.uint8_to_bits_little_endian(item_message.fireMessage.flags)
    # Extract the bytes containing the image data
    image_payload = item_payload[
                    item_message.imageOffset:item_message.imageOffset + item_message.imageSize]
    # Extract the bytes containing the bearing data
    bearing_payload = item_payload[
                      size:size + item_message.nBeams * 2]
    gain_payload = None
    # third bit of flags: won't send gain (0), will send gain (1)
    if flags[2]:
        offset_bytes = 4  # the gain is attached at the start of each range row, the gain is 4 bytes
        # calculate the number of bytes in each row
        strideBytes = oculus.enum_DataSizeType_to_size(
            item_message.dataSize) * item_message.nBeams + offset_bytes
        # get the gains associtated with the first 4 bytes of each row
        gain_payload = b"".join(image_payload[i:i+offset_bytes] for i in range(0, len(image_payload), strideBytes))
        # skip the first 4 bytes of each row
        image_payload = b"".join(image_payload[i + offset_bytes:i + strideBytes] for i in
                                 range(0, len(image_payload), strideBytes))

    polar_image = np.frombuffer(image_payload, dtype=np.uint8).reshape(item_message.nRanges, item_message.nBeams)
    bearing_table = np.frombuffer(bearing_payload, dtype=np.int16).astype(float) / 100.0 * (np.pi / 180.0)
    range_table = np.linspace(item_message.rangeResolution, item_message.nRanges * item_message.rangeResolution,
                              item_message.nRanges)
    # TODO Figure out if the gain should be something else
    gain_table = np.frombuffer(gain_payload, np.uint32) if gain_payload is not None else np.full((item_message.nRanges,), np.NAN, np.float32)
    return oculus.OculusPolarImage(polar_image, bearing_table, range_table, gain_table)


def polar_to_cart(polar_image: oculus.OculusPolarImage) -> oculus.OculusCartImage:
    """
    Converts an oculus.OculusPolarImage object into a oculus.OculusCartImage object.
    :param polar_image: contains polar coordinate image (with ranges and bearings)
    :return: cart_image: contains cartesian coordinate image (with
    """
    #warnings.warn("polar_to_cart is deprecated. Please use polar_to_cart2.", DeprecationWarning, stacklevel=2)
    azimuth = polar_image.bearing_table  # azimuth measured in radians from y-axis
    num_beams = len(azimuth)  # number of beams
    ranges = polar_image.ranging_table  # the range samples in m
    num_samples = len(ranges)  # the number of range samples

    azimuth_bounds = (azimuth.min(), azimuth.max())  # minimum and maximum bounds of the azimuth
    minus_width = np.floor(num_samples * np.sin(azimuth_bounds[0]))  # horizontal displacement in pixels in -ive direction
    plus_width = np.ceil(num_samples * np.sin(azimuth_bounds[1]))  # horizontal displace in pixels in +ive direction
    width = int(plus_width - minus_width)  # cartesian image width
    originx = np.abs(minus_width)  # the location of the polar image centre in x
    img_size = (num_samples, width)  # the cartesian image size, number of samples high x azimuthal extents
    newmap = np.zeros(img_size + (2,), np.float32)  # the mapping array must be the size of the cartesian image
    delta_azimuth = (azimuth_bounds[1] - azimuth_bounds[0]) / num_beams  # beam width in radians

    x, y = np.meshgrid(np.arange(newmap.shape[1]), np.arange(newmap.shape[0]))  # the x & y coordinates of the cartesian image
    delta_x = x - originx  # make sure x is referenced to the polar centre
    delta_y = newmap.shape[0] - y  # flip the y-axis (note this maybe shouldn't be done.
    R = np.sqrt(delta_x ** 2 + delta_y ** 2)  # calculate the radius for each cell from the centre
    azimuth = np.arctan2(delta_x, delta_y)  # calculate the azimuth given the x and y coordinates
    xp = (azimuth - azimuth_bounds[0]) / delta_azimuth  # the azimuth angle reference to the minimum azimuth, divided by the beam-width (pixels)
    yp = R.copy()  # the radius is mapped ot the y-axis directly of the polar image
    newmap[:, :, 0] = xp  # x coordinate mappings dst[x] = src[map[x]]
    newmap[:, :, 1] = yp  # y coordinate mappings dst[y] = src[map[y]]
    y = polar_image.ranging_table[:, None] * np.cos(polar_image.bearing_table)
    x = polar_image.ranging_table[:, None] * np.sin(polar_image.bearing_table)
    # do the remappings.
    cart_image = cv2.remap(polar_image.polar_image, newmap, None, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return oculus.OculusCartImage(cart_image, x, y)


def unpack_ping(item_payload: bytes) -> Union[oculus.OculusSimplePingResult, oculus.OculusSimplePingResult2, None]:
    """
    Given a bytes object, attempt to unpack a SimplePingResult (v1 or v2) structure.
    :param item_payload: the item_message's payload as a bytes object
    :return: either an oculus.OculusSimplePingResult or oculus.OculusSimplePingResult2 struct.
    """
    # Extract the header
    head = oculus.OculusMessageHeader.unpack(item_payload[:oculus.OculusMessageHeader.size()])
    if head.oculusId != oculus.OCULUS_CHECK_ID:
        raise ValueError(f"OculusID not {0xf453}, got {head.oculusId}")

    if head.msgId == oculus.OculusMessageType.messageSimplePingResult:
        ver = head.msgVersion
        if ver == 2:
            return oculus.OculusSimplePingResult2.unpack(item_payload[:oculus.OculusSimplePingResult2.size()])
        elif ver == 1:
            return oculus.OculusSimplePingResult.unpack(item_payload[:oculus.OculusSimplePingResult.size()])
        else:
            raise ValueError(f"Simple ping result version can only be 1 or 2. Got {ver}")
    elif head.msgId == oculus.OculusMessageType.messagePingResult:
        return oculus.OculusReturnFireMessage.unpack(item_payload[:oculus.OculusReturnFireMessage.size()])
    elif head.msgId == oculus.OculusMessageType.messageDummy:
        return None
    else:
        raise NotImplementedError(f"Oculus Message Type {oculus.OculusMessageType(head.msgId)}")


def get_header(entry: bytes) -> oculus.OculusMessageHeader:
    return oculus.OculusMessageHeader.unpack(entry[:oculus.OculusMessageHeader.size()])


def unpack_data_entry(entry: bytes) -> Union[Tuple[Union[oculus.OculusSimplePingResult, oculus.OculusSimplePingResult2], oculus.OculusPolarImage, bytes], None]:
    ping_result = unpack_ping(entry)
    if ping_result is None:
        return None, None, None
    polar_image_data = parse_polar_image(entry, ping_result)
    ping_result = filter_gain_result(ping_result)
    new_buffer = ping_result.pack() + entry[
                                      ping_result.size():ping_result.imageOffset] + polar_image_data.polar_image.tobytes()
    return ping_result, polar_image_data, new_buffer


@dataclass
class ItemInfo:
    entryId: int
    timestamp: float
    type: int
    dataSourceId: int
