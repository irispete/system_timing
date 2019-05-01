
#include <iris_unit_test_framework/iris_unit_test_framework.h>
#include <iris_common/log/csv_logger_inc.h>
#define private public
#include <system_timing/event_time_publisher.h>
#include <system_timing/event_time_manager.h>
#include <iris_common/test/mock_logger.h>
#undef private

using namespace system_timing;
using ::testing::_;
using ::testing::Return;


class EventTimeManagerTests : public testing::Test
{
    public:
        MockLogger mock_logger_{};
        EventTimeManager manager_{mock_logger_};

        void SetUp()
        {
            manager_.setSaveFunction(std::bind(&EventTimeManagerTests::saveSequence, this, std::placeholders::_1));
        }

        void TearDown()
        {
            manager_.setSaveFunction(nullptr);
        }

        void saveSequence(std::list<iris_common::EventTime> &sequence)
        {
            sequence_ = sequence;
            call_count_++;
        }

        iris_common::EventTime makeEvent(const std::string &node, const std::string &event, const std::string &category, int id, EventTimeType event_type, int time)
        {
            iris_common::EventTime event_time{};
            event_time.node.data = node;
            event_time.event.data = event;
            event_time.category.data = category;
            event_time.id = id;
            event_time.type = static_cast<int>(event_type);
            event_time.time = time;
            return std::move(event_time);
        }

        int call_count_ = 0;
        std::list<iris_common::EventTime> sequence_;

};

/**
 * @brief
 */
TEST_F(EventTimeManagerTests, AddEndEventTest)
{
    // should add the event and get rid of sequence, one error due to no existing sequence
    auto event = makeEvent("node", "end", "EO", 111, EventTimeType::end, 123);
    manager_.addEvent(event);
    ASSERT_EQ(call_count_, 1);
    ASSERT_EQ(mock_logger_.error_count, 1);
    auto sequences_it = manager_.sequences_.find("EO");
    ASSERT_NE(sequences_it, manager_.sequences_.end());
    // the sequence should be deleted
    auto sequence_it = sequences_it->second.find(111);
    ASSERT_EQ(sequence_it, sequences_it->second.end());
}

TEST_F(EventTimeManagerTests, AddAddEventTest)
{
    // should add the event, one error due to no existing sequence
    auto event = makeEvent("node", "add", "EO", 111, EventTimeType::add, 123);
    manager_.addEvent(event);
    auto sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 1u);
    ASSERT_EQ(mock_logger_.error_count, 1);
    
    // should add the event and get rid of sequence, one error due to lesser time
    event = makeEvent("node", "end", "EO", 111, EventTimeType::end, 111);
    manager_.addEvent(event);
    ASSERT_EQ(sequence_.size(), 2u);
    ASSERT_EQ(mock_logger_.error_count, 2);
}

TEST_F(EventTimeManagerTests, AddTwoEventTest)
{
    // should add the first node event free
    auto event = makeEvent("node", "first", "EO", 111, EventTimeType::start, 123);
    manager_.addEvent(event);
    auto sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 1u);
    ASSERT_EQ(mock_logger_.error_count, 0);

    // should clear the list and add the event with one error
    event = makeEvent("node", "first", "EO", 111, EventTimeType::start, 123);
    manager_.addEvent(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 1u);
    ASSERT_EQ(mock_logger_.error_count, 1);

    // should add the event
    event = makeEvent("node", "add", "EO", 111, EventTimeType::add, 222);
    manager_.addEvent(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 2u);
    ASSERT_EQ(mock_logger_.error_count, 1);

    // should add the event but log an error because the time is out of sequence
    event = makeEvent("node", "add", "EO", 111, EventTimeType::add, 133);
    manager_.addEvent(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 3u);
    ASSERT_EQ(mock_logger_.error_count, 2);

    // should add the event and get rid of sequence
    event = makeEvent("node", "end", "EO", 111, EventTimeType::end, 333);
    manager_.addEvent(event);
    ASSERT_EQ(sequence_.size(), 4u);
    ASSERT_EQ(mock_logger_.error_count, 2);
    auto sequences_it = manager_.sequences_.find("EO");
    ASSERT_NE(sequences_it, manager_.sequences_.end());
    // the sequence should be deleted
    auto sequence_it = sequences_it->second.find(111);
    ASSERT_EQ(sequence_it, sequences_it->second.end());
}

/**
 * @brief
 */
TEST_F(EventTimeManagerTests, FindSequenceTest)
{
    auto event = makeEvent("node", "first", "EO", 111, EventTimeType::start, 123);
    auto sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 0u);
    // should find the sequence
    manager_.sequences_[event.category.data][event.id].push_back(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 1u);

    // shouldn't find the sequence
    event = makeEvent("node", "first", "adsb", 222, EventTimeType::start, 123);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence_.size(), 0u);
    manager_.sequences_[event.category.data][event.id].push_back(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 1u);
    event = makeEvent("node", "add", "adsb", 222, EventTimeType::add, 234);
    manager_.sequences_[event.category.data][event.id].push_back(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 2u);

    event = makeEvent("node", "first", "adsb", 333, EventTimeType::start, 123);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence_.size(), 0u);
    manager_.sequences_[event.category.data][event.id].push_back(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 1u);

    manager_.flush();
    ASSERT_EQ(manager_.sequences_.size(), 0u);
    ASSERT_EQ(call_count_, 3);
}

/**
 * @brief
 */
TEST_F(EventTimeManagerTests, CheckAndFlushOldEventsTest)
{
    // add a bunch of sequences
    for (int i = 1; i <= manager_.MAX_SEQUENCES+2; i++)
    {
        auto event = makeEvent("node", "event", "EO", i, EventTimeType::start, 123);
        manager_.addEvent(event);
    }
    ASSERT_EQ(call_count_, 2);

    ASSERT_EQ(mock_logger_.error_count, 0);
    auto sequences_it = manager_.sequences_.find("EO");
    ASSERT_NE(sequences_it, manager_.sequences_.end());
    auto key = sequences_it->first;
    auto value = sequences_it->second;
    // the sequence should be deleted
    auto sequence_it = sequences_it->second.find(1);
    ASSERT_EQ(sequence_it, sequences_it->second.end());
    // the sequence should be deleted
    sequence_it = sequences_it->second.find(2);
    ASSERT_EQ(sequence_it, sequences_it->second.end());
    // should still be MAX_SEQUENCES
    ASSERT_EQ(sequences_it->second.size(), static_cast<unsigned long>(manager_.MAX_SEQUENCES));
}
