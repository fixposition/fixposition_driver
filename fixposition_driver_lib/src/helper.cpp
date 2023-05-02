/**
 *  @file
 *  @brief Helper functions
 *
 * \verbatim
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 * \endverbatim
 *
 */

/* PACKAGE */
#include <fixposition_driver_lib/helper.hpp>
#include <fixposition_driver_lib/nov_type.hpp>

namespace fixposition {

static constexpr const char kNmeaPreamble = '$';
static constexpr const int kLibParserMaxNmeaSize = 400;

void SplitMessage(std::vector<std::string>& tokens, const std::string& msg, const std::string& delim) {
    boost::split(tokens, msg, boost::is_any_of(delim));
}

void BestGnssPosToNavSatFix(const Oem7MessageHeaderMem* const header, const BESTGNSSPOSMem* const bestgnsspos,
                            NavSatFixData& navsatfix) {
    navsatfix.latitude = bestgnsspos->lat;
    navsatfix.longitude = bestgnsspos->lon;
    navsatfix.altitude = bestgnsspos->hgt;

    Eigen::Array3d cov_diag(bestgnsspos->lat_stdev, bestgnsspos->lon_stdev, bestgnsspos->hgt_stdev);

    navsatfix.cov = (cov_diag * cov_diag).matrix().asDiagonal();
    navsatfix.position_covariance_type = 2;

    switch (static_cast<PositionOrVelocityType>(bestgnsspos->pos_type)) {
        case PositionOrVelocityType::NARROW_INT:
            navsatfix.status.status = static_cast<uint8_t>(NavSatStatusData::Status::STATUS_GBAS_FIX);
            break;
        case PositionOrVelocityType::NARROW_FLOAT:
        case PositionOrVelocityType::SINGLE:
            navsatfix.status.status = static_cast<uint8_t>(NavSatStatusData::Status::STATUS_FIX);
            break;
        default:
            navsatfix.status.status = static_cast<uint8_t>(NavSatStatusData::Status::STATUS_NO_FIX);
    }

    // TODO hardcoded for now for all 4 systems
    navsatfix.status.service = 0b000000000000000;
    navsatfix.status.service |= 1;
    navsatfix.status.service |= 2;
    navsatfix.status.service |= 4;
    navsatfix.status.service |= 8;

    switch (static_cast<MessageTypeSource>(header->message_type & static_cast<uint8_t>(MessageTypeSource::_MASK))) {
        case MessageTypeSource::PRIMARY:
            navsatfix.frame_id = "GNSS1";
            break;
        case MessageTypeSource::SECONDARY:
            navsatfix.frame_id = "GNSS2";
            break;
        case MessageTypeSource::_MASK:
            navsatfix.frame_id = "GNSS";
            break;
        default:
            navsatfix.frame_id = "GNSS";
    }
}

template <>
void NovToData<BESTGNSSPOSMem, NavSatFixData>(const Oem7MessageHeaderMem* const header, const BESTGNSSPOSMem* const nov,
                                              NavSatFixData& data) {
    BestGnssPosToNavSatFix(header, nov, data);
}

}  // namespace fixposition
