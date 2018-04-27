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
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
import logging
import rospy

import actionlib
from conversation_engine.msg import ConverseAction, ConverseGoal, ConverseFeedback, ConverseResult

class ConversationEngineBot(object):

    def __init__(self, token, robot_name):
        self.ac = actionlib.SimpleActionClient("/conversation_engine", ConverseAction)
        self.ac.wait_for_server()

        # Create the EventHandler and pass it your bot's token.
        self.updater = Updater(token)

        # Get the dispatcher to register handlers
        self.dp = self.updater.dispatcher

        # on different commands - answer in Telegram
        self.dp.add_handler(CommandHandler("start", self._start))
        self.dp.add_handler(CommandHandler("help", self._help))

        # on noncommand i.e message - echo the message on Telegram
        self.dp.add_handler(MessageHandler(Filters.text, self._accept_command))

        # log all errors
        self.dp.add_error_handler(self._error)

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