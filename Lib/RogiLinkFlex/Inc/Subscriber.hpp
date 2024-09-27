#pragma once

#include "Serializer.hpp"
#include <functional>
#include <utility>

// // applyを使うためのヘルパー関数
// template<typename Function, typename Tuple, size_t... Indices>
// auto apply_impl(Function f, Tuple&& t, std::index_sequence<Indices...>) -> decltype(f(std::get<Indices>(std::forward<Tuple>(t))...)) {
//     return f(std::get<Indices>(std::forward<Tuple>(t))...);
// }

// // std::applyの代替(C++17未満)
// template<typename Function, typename Tuple>
// auto apply(Function f, Tuple&& t) -> decltype(apply_impl(f, std::forward<Tuple>(t), std::make_index_sequence<std::tuple_size<typename std::remove_reference<Tuple>::type>::value>{})) {
//     return apply_impl(f, std::forward<Tuple>(t), std::make_index_sequence<std::tuple_size<typename std::remove_reference<Tuple>::type>::value>{});
// }

// template<typename Function, typename Tuple, size_t... Indices>
// auto apply_impl(Function f, Tuple&& t, std::index_sequence<Indices...>) -> decltype(f(std::get<Indices>(std::forward<Tuple>(t))...)) {
//     return f(std::get<Indices>(std::forward<Tuple>(t))...);
// }

// template<typename Function, typename Tuple>
// auto original_apply(Function f, Tuple&& t) -> decltype(apply_impl(f, std::forward<Tuple>(t), std::make_index_sequence<std::tuple_size<typename std::remove_reference<Tuple>::type>::value>{})) {
//     return apply_impl(f, std::forward<Tuple>(t), std::make_index_sequence<std::tuple_size<typename std::remove_reference<Tuple>::type>::value>{});
// }

// Subscriberクラス。Communicationクラスとコールバックを受け取り、受信データをデシリアライズしてコールバックを呼び出す
template <typename Communication, typename... Args>
class Subscriber {
    public:
        Subscriber(Communication& communication, uint8_t frame_id) : communication(communication) {
            if (frame_id == 0) {
                return;
            }

            this->frame_id = frame_id;

            this->callback = [](Args...){}; // デフォルトコールバック
            
            // コールバックを登録
            communication.add_callback(frame_id, [this](uint8_t* buffer, size_t size) {
                this->on_receive(buffer, size);
            });
        }

        // コールバックを登録
        void set_callback(std::function<void(Args...)> callback){
            this->callback = callback;
        }

        // 受信時の処理
        void on_receive(uint8_t* buffer, size_t size) {
            std::tuple<Args...> args;
            deserializer.deserialize_tuple(buffer, args, size); // デシリアライズ

            std::apply(callback, args); // ここでコールバックを呼び出す
        }

    private:
        Communication& communication;
        TupleDeserializer<Args...> deserializer;
        uint8_t frame_id;

        std::function<void(Args...)> callback;
};