#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Simple Bot to reply to Telegram messages
# This program is dedicated to the public domain under the CC0 license.
"""
This Bot uses the Updater class to handle the bot.

First, a few handler functions are defined. Then, those functions are passed to
the Dispatcher and registered at their respective places.
Then, the bot is started and runs until we press Ctrl-C on the command line.

Usage:
Basic Echobot example, repeats messages.
Press Ctrl-C on the command line or send a signal to the process to stop the
bot.
"""
from telegram import Bot
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
import logging
import rospy

import actionlib
from conversation_engine.msg import ConverseAction, ConverseGoal, ConverseFeedback, ConverseResult

from hmi import AbstractHMIServer, HMIResult
from hmi.common import parse_sentence

from threading import Event

class ConversationEngineBot(AbstractHMIServer):

    def __init__(self, token, robot_name):
        super(self.__class__, self).__init__("conversation_engine_hmi")

        self.robot_name = robot_name

        self.ac = actionlib.SimpleActionClient("/conversation_engine", ConverseAction)
        self.ac.wait_for_server()

        self._bot = None
        self._chat_id = None
        self._wait_for_answer = Event()

        # Create the EventHandler and pass it your bot's token.
        self.updater = Updater(token)

        # Get the dispatcher to register handlers
        self.dp = self.updater.dispatcher

        # on different commands - answer in Telegram
        self.dp.add_handler(CommandHandler("start", self._start))
        self.dp.add_handler(CommandHandler("help", self._help))
        self.dp.add_handler(CommandHandler("answer", self._answer_question))

        # on noncommand i.e message - echo the message on Telegram
        self.dp.add_handler(MessageHandler(Filters.text, self._accept_command))

        # log all errors
        self.dp.add_error_handler(self._error)

    # Part of AbstractHmiServer
    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        # Pose the question to the user via the chat
        self._bot.send_message(chat_id=self._chat_id,
                               text=description+" (Prefix your answer with /answer")

        self._wait_for_answer.wait()

        if rospy.is_shutdown() or is_preempt_requested():
            return None

        rospy.loginfo("Received string: '%s'", self._string)

        stripped = str(self._string.replace("/answer ", ""))

        semantics = parse_sentence(stripped, grammar, target, debug=True)

        rospy.loginfo("Parsed semantics: %s", semantics)

        result = HMIResult(stripped, semantics)
        self._string = None
        self._wait_for_answer.clear()

        return result

    def run(self):
        # Start the Bot
        self.updater.start_polling()

    def stop(self):
        self.updater.stop()

    # Define a few command handlers. These usually take the two arguments bot and
    # update. Error handlers also receive the raised TelegramError object in error.
    def _start(self, bot, update):
        rospy.loginfo("Received {}".format(update.message.text))
        update.message.reply_text('Hi! Your typed wish is my command')

        assert isinstance(bot, Bot)
        self._bot = bot
        self._chat_id = update.message.chat_id

        self._bot.send_message(chat_id=update.message.chat_id,
                               text="I'm {}, please talk to me!".format(self.robot_name))

    def _help(self, bot, update):
        rospy.loginfo("Received {}".format(update.message.text))
        update.message.reply_text('With what? Please type a command in "natural" language')

    def _accept_command(self, bot, update):
        rospy.loginfo("Received {}".format(update.message.text))

        # update.message.reply_text(update.message.text)

        goal = ConverseGoal(command=update.message.text)
        rospy.loginfo(goal)

        self.ac.send_goal(goal)
        self.ac.wait_for_result()
        result = self.ac.get_result()

        rospy.loginfo(result)

        update.message.reply_text(result.result_sentence)

    def _answer_question(self, bot, update):
        self._string = update.message.text
        self._wait_for_answer.set()

    def _error(self, bot, update, error):
        rospy.logerr('Update "%s" caused error "%s"' % (update, error))


if __name__ == '__main__':
    rospy.init_node('telegram_example')
    token = rospy.get_param('/telegram/token', None)
    robot_name = rospy.get_param('/robot/name', "amigo")

    if token is None:
        rospy.logerr("No token found in /telegram/token param server.")
        exit(0)

    engine = ConversationEngineBot(token, robot_name)

    engine.run()

    rospy.spin()  # Use the rospy mainloop

    engine.stop()
