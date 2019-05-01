#include <iris_common/EventTime.h>
#include <iris_common/log/csv_logger.h>

#ifndef __CAS_LOG_TIMER_LOG__
#define __CAS_LOG_TIMER_LOG__

namespace system_timing
{
    /**
     * @brief   A specific CSV logger for Timer.
     */
    class EventTimeCsvLogger : public iris_common::CsvLogger<std::list<iris_common::EventTime>>
    {
        public:
            /**
             * @brief initialize. Calls the parent initialize()r for actual work.
             * @param file_name     File to save to.
             */
            void initialize(const std::string &file_name) override;

            /**
             * Writes a structure of values to the file.
             * @param  values   Structure to write.
             * @return          true on success, false otherwise
             */
            bool write(const std::list<iris_common::EventTime>& sequence) const override;
    };
}

#endif /* __CAS_LOG_TIMER_LOG__ */
