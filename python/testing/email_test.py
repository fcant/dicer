import smtplib

from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

numbers = [1,2,3,4,5,6,77,88,99]

server = smtplib.SMTP('mail.gmx.net', 587)
server.starttls()


server.login('python-email@gmx.de', 'bojack123.')

msg = MIMEMultipart()
msg['From'] = 'python-email@gmx.de'
msg['To'] = 'zug209@gmx.net'
msg['Subject'] = 'Testreihe fertig'
message = str(numbers[0]) + ',' + str(numbers[1]) + ',' + str(numbers[2]) + ',' + str(numbers[3]) + ',' + str(numbers[4]) + ',' + str(numbers[5]) + ' Err: ' + str(numbers[6]) + ' All: ' + str(numbers[6]) + ' Std: ' + str(numbers[7])
msg.attach(MIMEText(message))

# send the message via the server set up earlier.
server.send_message(msg)



#server.sendmail('python-email@gmx.de', 'zug209@gmx.net', 'YourMessage')

print('fertig')