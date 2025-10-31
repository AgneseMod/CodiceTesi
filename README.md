Questo file è il codice che ho usato per il mio progetto di laurea dove ho lavorato con una scheda SX1262 826M LoRa HAT. 
Parte è codice demo preso dal sito Waveshare, la parte da me realizzata è la parte di invio di immagini che gestisce sia il trasmettitore che il ricevitore.
Il codice divide il file immagine in diversi pacchetti a cui ad ognuno di esso viene aggiunto un header con informazioni utili all'invio del pacchetto (indirizzo destinatario, mittente, offset di frequenza, numero di pacchetti totali ed un marker per indicare il tipo di pacchetto)
Il ricevitore salva i pacchetti in un buffer legge l'header ed in base al marker capisce quale tipo di pacchetto sia così il dato viene ricomposto
Inoltre ho creato un meccanismo di conferma di ricezione tramite ACK

SI TRATTA DI UN LAVORO IN CORSO:
il codice nonostante logicamente valido non funziona, ci sono alcuni problemi di gestione del buffer della seriale e problemi di sincronizzazione ACK. 
