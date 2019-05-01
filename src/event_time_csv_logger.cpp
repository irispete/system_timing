#include <system_timing/event_time_csv_logger.h>
#include <iris_common/log/csv_logger_inc.h>

namespace system_timing
{

    void EventTimeCsvLogger::initialize(const std::string &file_name)
    {
        std::vector<std::string> headers;
        // enough for 12 events
        for (int i = 0;i < 12; i++)
        {
            headers.insert(headers.end(),
                    {
                            "Node",
                            "Event",
                            "Cat",
                            "ID",
                            "Type",
                            "Time"
                    });
        }
        CsvLogger::initialize(file_name,headers);
    }

    bool EventTimeCsvLogger::write(const std::list<iris_common::EventTime>& sequence) const
    {
        assert(CsvLogger::is_initialized_);

        if (!output_.is_open())
        {
            return false;
        }

        bool is_first = true;
        for (auto event : sequence)
        {
            if (!is_first)
            {
                output_ << VALUE_SEPARATOR;
            }
            is_first = false;
            output_ << event.node.data << VALUE_SEPARATOR;
            output_ << event.event.data << VALUE_SEPARATOR;
            output_ << event.category.data << VALUE_SEPARATOR;
            output_ << event.id << VALUE_SEPARATOR;
            output_ << event.type << VALUE_SEPARATOR;
            output_ << event.time;
        }

        output_ << ENTRY_SEPARATOR;
        output_.flush();

        return true;
    }
}
