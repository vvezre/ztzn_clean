# -*- coding: utf-8 -*-
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.cron import CronTrigger
import threading
import logging

logging.basicConfig(level=logging.INFO)

class DynamicCronScheduler(object):
    def __init__(self):
        self.scheduler = BackgroundScheduler()
        self.scheduler.start()
        self._lock = threading.Lock()

    def add_job(self, job_id, func, cron):
        """
        添加任务（Python 2 兼容）
        :param job_id: str
        :param func: callable
        :param cron: str, e.g., "*/5 * * * *"
        """
        trigger = CronTrigger.from_crontab(cron)
        with self._lock:
            self.scheduler.add_job(
                func=func,
                trigger=trigger,
                id=job_id,
                replace_existing=True
            )
        logging.info("Added job %s with cron: %s", job_id, cron)

    def update_cron(self, job_id, new_cron):
        """动态更新 cron"""
        trigger = CronTrigger.from_crontab(new_cron)
        with self._lock:
            self.scheduler.reschedule_job(job_id=job_id, trigger=trigger)
        logging.info("Updated job %s to cron: %s", job_id, new_cron)

    def shutdown(self, wait=True):
        self.scheduler.shutdown(wait=wait)
        logging.info("Scheduler shut down.")