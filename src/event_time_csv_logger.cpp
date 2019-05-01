#include <chrono>
#include <ctime>
#include <system_timing/event_time_csv_logger.h>
#include <iris_common/log/csv_logger_inc.h>

namespace system_timing
{

    void EventTimeCsvLogger::initialize(const std::string &file_name)
    {
        std::vector<std::string> headers;
        headers.insert(headers.end(),
               {
                       "Cat",
                       "ID"
               });
        // enough for 12 events
        for (int i = 0;i < 12; i++)
        {
            headers.insert(headers.end(),
                    {
                            "Node",
                            "Event",
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
            else
            {
                is_first = false;
                output_ << event.category.data << VALUE_SEPARATOR;
                output_ << event.id << VALUE_SEPARATOR;
            }
            output_ << event.node.data << VALUE_SEPARATOR;
            output_ << event.event.data << VALUE_SEPARATOR;
            output_ << event.type << VALUE_SEPARATOR;
            char time_str[20];
            std::chrono::milliseconds duration(event.time);
            std::chrono::time_point<std::chrono::system_clock> time(duration);
            std::time_t t = std::chrono::system_clock::to_time_t(time);
            const std::chrono::duration<double> tse = time.time_since_epoch();
            std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count() % 1000;
            strftime(time_str, 20, "%H:%M:%S", localtime(&t));
            output_ << time_str << "." << milliseconds;
        }

        output_ << ENTRY_SEPARATOR;
        output_.flush();

        return true;
    }
}
