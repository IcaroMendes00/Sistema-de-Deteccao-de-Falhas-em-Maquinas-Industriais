import serial
import csv
import time
import sys
import threading

# Configuração da UART (ajuste para a porta correta)
ser = serial.Serial('COM3', 115200, timeout=1)

# Nome do arquivo CSV
csv_filename = "mpu6050_data_anom.csv"

# Solicita a quantidade de linhas a serem coletadas
num_lines = int(input("Digite a quantidade de linhas a serem coletadas: "))

# Inicializa variáveis de controle
line_count = 0
start_time = time.time()
buffer = ""  # Buffer para armazenar fragmentos de dados
spinner_running = True

# Função do spinner para mostrar carregamento
def spinner():
    while spinner_running:
        for char in "|/-\\":
            sys.stdout.write(f"\r{char} Coletando... {line_count}/{num_lines} linhas")
            sys.stdout.flush()
            time.sleep(0.1)

# Inicia o spinner em uma thread separada
spinner_thread = threading.Thread(target=spinner)
spinner_thread.start()

print("\nIniciando a coleta de dados... Pressione Ctrl+C para interromper.")

# Abre o arquivo CSV para gravação
with open(csv_filename, "w", newline='') as file:
    writer = csv.writer(file, delimiter=' ')

    while line_count < num_lines:
        try:
            # Lê qualquer dado disponível na UART
            raw_data = ser.read(ser.in_waiting or 1).decode('utf-8', errors='ignore')

            # Adiciona os dados recebidos ao buffer
            buffer += raw_data.replace("\n", " ")  # Substitui quebras de linha por espaço

            # Divide os dados no buffer para verificar se temos 768 valores completos
            data = buffer.split()

            if len(data) >= 768:
                valid_data = data[:768]  # Mantém apenas os 768 primeiros valores
                writer.writerow(valid_data)  # Salva no CSV
                line_count += 1  # Atualiza o contador de linhas válidas
                print(f"\n{line_count} linha(s) adicionada(s)")

                # Remove os dados processados do buffer
                buffer = ' '.join(data[768:])

        except KeyboardInterrupt:
            print("\nCaptura interrompida pelo usuário.")
            spinner_running = False
            spinner_thread.join()
            break

# Finaliza a coleta e exibe estatísticas
spinner_running = False
spinner_thread.join()

end_time = time.time()
total_time = end_time - start_time
average_time_per_line = total_time / line_count if line_count > 0 else 0

print("\nColeta finalizada!")
print(f"Total de linhas coletadas: {line_count}/{num_lines}")
print(f"Tempo total de coleta: {total_time:.2f} segundos")
print(f"Tempo médio por linha: {average_time_per_line:.2f} segundos")
print(f"Arquivo salvo: {csv_filename}")

ser.close()  # Fecha a conexão serial
