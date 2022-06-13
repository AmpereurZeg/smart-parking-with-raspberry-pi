import sqlite3

conn = sqlite3.connect('matricule.db')
c=conn.cursor()
c.execute("""CREATE TABLE plaque (
	    id integer primary key autoincrement,
	    matricule text) """)
c.execute("INSERT INTO plaque VALUES (1,'HR 26 DA 2330')")
conn.commit()
conn.close()
